#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <libesp.h>
#include <string.h>

#include "private.h"

#define ALWAYS_INLINE __attribute__((always_inline)) inline

struct rfm69hcw_irq_task {
    // *** Read-only part ***
    struct irq_task_ctrl {
        rfm69hcw_handle_t dev;

        // A copy of this pin number must be stored here for the ISR to access.
        gpio_num_t pin_irq;

        // The external handler to invoke when the IRQ occurs.
        bool (*handle_irq)(rfm69hcw_irq_task_handle_t task, rfm69hcw_handle_t dev);

        // Used (by the irq task only) to wait for a signal from the
        // IRQ ISR.
        SemaphoreHandle_t irq_sem;

        // Used to protect the `irq_task_state` member, below.
        SemaphoreHandle_t state_mutex;
    } ctrl;

    // *** Read-write part, protected by `irq_task_ctrl.state_mutex` ***
    struct irq_task_state {
        // This member is initially set to `NULL`. In order to signal the
        // irq task to terminate it is set to a nonnull value, and then
        // once shutdown of the irq task has progressed sufficiently
        // that it can be guarenteed that no further physical IO with the
        // RFM69HCW will be peformed, this semaphore is "given". It is then
        // the responsibility of the task waiting on this semaphore to
        // destory it.
        SemaphoreHandle_t should_stop;
    } state;
};

typedef struct irq_task_ctrl irq_task_ctrl_t;
typedef struct irq_task_state irq_task_state_t;

static rfm69hcw_irq_task_t* alloc_irq_task(rfm69hcw_handle_t dev, bool (*handle_irq)(rfm69hcw_irq_task_handle_t task, rfm69hcw_handle_t dev)) {
    irq_task_ctrl_t ctrl = {
        .dev = dev,
        .pin_irq = dev->pin_irq,
        .handle_irq = handle_irq,
        .irq_sem = xSemaphoreCreateBinary(),
        .state_mutex = xSemaphoreCreateMutex(),
    };

    irq_task_state_t state = {
        .should_stop = NULL,
    };

    rfm69hcw_irq_task_t* task = malloc(sizeof(rfm69hcw_irq_task_t));
    task->ctrl = ctrl;
    task->state = state;

    return task;
}

static void free_irq_task(rfm69hcw_irq_task_t* task) {
    vSemaphoreDelete(task->ctrl.irq_sem);
    vSemaphoreDelete(task->ctrl.state_mutex);

    // Note that we do not free `task->state.should_stop`,
    // since that is the responsibility of the waiting task
    // (and it might not have woken yet).

    free(task);
}

static void isr_irq(void* arg) {
    rfm69hcw_irq_task_t* task = arg;

    // First, since this interrupt is level-triggered (so we don't miss any) disable further
    // interrupts. If the `irq_loop()` happens to enable this interrupt again asynchronously
    // with this, we don't mind, since `sleep_until_irq_high()` clears spurious pending
    // messages.
    gpio_intr_disable(task->ctrl.pin_irq);

    // Only now give the semaphore, which will ensure that the `irq_loop()` will always
    // eventually take the semaphore and call `gpio_intr_enable()` for us. (We don't care
    // about the return value, i.e. whether we were able to.)
    xSemaphoreGiveFromISR(task->ctrl.irq_sem, NULL);
}

static ALWAYS_INLINE void sleep_until_irq_high(const rfm69hcw_irq_task_t* task) {
    // Clear a potential spurious message from the IRQ ISR.
    xSemaphoreTake(task->ctrl.irq_sem, 0);

    // If we are here, IRQ is low. First, enable the IRQ ISR. Since the ISR disables
    // itself before it is possible to take `player->irq_sem`, a race condition cannot
    // occur here.
    gpio_intr_enable(task->ctrl.pin_irq);

    // Wait until the ISR tells us IRQ has gone high again.
    while (xSemaphoreTake(task->ctrl.irq_sem, portMAX_DELAY) == pdFALSE)
        ;

    // The concurrency "invariant" at this point is that we are guarenteed that the ISR
    // handler is disabled.
}

static void irq_loop(rfm69hcw_irq_task_t* task) {
    irq_task_ctrl_t ctrl = task->ctrl;

    while (1) {
        while (xSemaphoreTake(ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
            ;

        SemaphoreHandle_t should_stop = task->state.should_stop;

        xSemaphoreGive(ctrl.state_mutex);

        if (should_stop) {
            ESP_LOGD(TAG, "loop stopping");
            xSemaphoreGive(should_stop);
            return;
        }

        sleep_until_irq_high(task);

        if (!task->ctrl.handle_irq(task, task->ctrl.dev)) {
            // The registered function has decided we should stop.
            return;
        }
    }
}

static void irq_task(void* arg) {
    rfm69hcw_irq_task_t* task = arg;

    gpio_num_t pin_irq = task->ctrl.pin_irq;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_HIGH_LEVEL,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pin_irq),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Prevent the level trigger interrupt on the IRQ pin
    // from hanging up the core when we enable interrupts.
    gpio_intr_disable(pin_irq);

    // TODO Install in a way which doesn't collide with other code.
    // (Note that we need `gpio_install_isr_service()` to have been
    // called without the flag which requires IRAM ISRs.)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin_irq, isr_irq, task);

    irq_loop(task);

    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    if (stack_remaining <= 0) {
        ESP_LOGE(TAG, "stack overflow detected!");
    }

    ESP_LOGD(TAG, "stack words remaining: %u", stack_remaining);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_isr_handler_remove(pin_irq);
    gpio_uninstall_isr_service();

    gpio_intr_enable(pin_irq);

    free_irq_task(task);

    ESP_LOGD(TAG, "irq task stopped");
}

// The player task is pinned to a core because ISRs must be
// deregistered on the same core as that on which they were registered.
#define RUN_ON_CORE_NUM 1

#define TASK_STACK_SIZE 4096

esp_err_t rfm69hcw_enter_rx(rfm69hcw_handle_t dev, const rfm69hcw_rx_config_t* cfg, bool (*handle_irq)(rfm69hcw_irq_task_handle_t task, rfm69hcw_handle_t dev), const BaseType_t core_id, const uint32_t task_task_size, rfm69hcw_irq_task_handle_t* out_handle) {
    rfm69hcw_configure_rx(dev, cfg);

    rfm69hcw_irq_task_t* task = alloc_irq_task(dev, handle_irq);

    // Note that this task will allocate an interrupt, and therefore must happen on a well-defined core
    // (hence `xTaskCreatePinnedToCore()`), since interrupts must be deallocated on the same core they
    // were allocated on.
    BaseType_t result = xTaskCreatePinnedToCore(&irq_task, "rfm69hcw_irq_task", TASK_STACK_SIZE, (void*) task,
                                                10, NULL, RUN_ON_CORE_NUM);
    if (result != pdPASS) {
        free_irq_task(task);

        ESP_LOGE(TAG, "failed to create irq task! (0x%X)", result);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "created irq task: %p", task);

    *out_handle = task;
    return ESP_OK;
}

void rfm69hcw_managed_enter_tx(rfm69hcw_handle_t dev /*, const rfm69hcw_tx_config_t* cfg*/) {
    ESP_LOGE(TAG, "unimplemented!");
    abort();

    // FIXME do this.

    // rfm69hcw_reg_write(dev, RFM69HCW_REG_OP_MODE, RFM69HCW_OP_MODE_MODE_STANDBY);
    // rfm69hcw_set_power_level(dev, RFM69HCW_POWER_????);

    // FIXME abstract RFM69HCW_REG_FIFO_THRESH
    // rfm69hcw_reg_write(dev, RFM69HCW_REG_FIFO_THRESH, 0x01);

    {
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_FDEV_MSB, 0x02);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_FDEV_LSB, 0x3E);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_PREAMBLE_MSB, 0x00);
        // rfm69hcw_reg_write(dev, RFM69HCW_REG_PREAMBLE_LSB, 0x06);

        // // FIXME how does the arduino library select this?
        // // rfm69hcw_reg_write(dev, RFM69HCW_REG_PA_RAMP, 0x0A);
    }
}

void rfm69hcw_wake(rfm69hcw_irq_task_handle_t task) {
    xSemaphoreGive(task->ctrl.irq_sem);
}

void rfm69hcw_destroy(rfm69hcw_irq_task_handle_t task) {
    ESP_LOGD(TAG, "destroying irq task: %p", task);

    // Because the irq task keeps a pointer to itself, at this point all we need to do is
    // to signal to the player task to stop, and wait for that to be acknowledged.
    SemaphoreHandle_t should_stop = xSemaphoreCreateBinary();

    // Take the state mutex.
    while (xSemaphoreTake(task->ctrl.state_mutex, portMAX_DELAY) == pdFALSE)
        ;

    // The `should_stop` signal can only be sent once, and no further
    // updates are then permitted. (Because allowing such would both
    // be useless in practice and also would create a race condition.)
    assert(!task->state.should_stop);
    task->state.should_stop = should_stop;

    // Give the state mutex.
    xSemaphoreGive(task->ctrl.state_mutex);

    // Wake the task if it is sleeping.
    rfm69hcw_wake(task);

    // Wait for the signal that stopping has sufficiently completed.
    while (xSemaphoreTake(should_stop, portMAX_DELAY) == pdFALSE)
        ;

    // Free the signally semaphore.
    vSemaphoreDelete(should_stop);
}
