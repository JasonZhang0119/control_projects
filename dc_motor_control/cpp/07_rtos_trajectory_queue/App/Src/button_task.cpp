
#include "task_wrapper.h"
#include "main.h"

static TaskHandle_t g_trajectory_task_handle = nullptr;

void Module_RegisterTrajectoryTaskHandle(void* handle){
    g_trajectory_task_handle = static_cast<TaskHandle_t>(handle);
}

extern "C" void ButtonTask(void* pv)
{
    auto* ctx = static_cast<SystemContext*>(pv);

    if (ctx->uart_logger != nullptr) {
        ctx->uart_logger->SendString("button task started\r\n");
    }

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));

        const ButtonEvent event =
            ctx->position_button->ConsumeEvent();

        switch (event) {
            case ButtonEvent::SpeedUp:
                ctx->position_ref += Config::kPositionRefStepRadS;

                if (ctx->position_ref > Config::kPositionRefMaxRadS) {
                    ctx->position_ref = Config::kPositionRefMaxRadS;
                }
                break;

            case ButtonEvent::SpeedDown:
                ctx->position_ref -= Config::kPositionRefStepRadS;

                if (ctx->position_ref < Config::kPositionRefMinRadS) {
                    ctx->position_ref = Config::kPositionRefMinRadS;
                }
                break;

            case ButtonEvent::None:
            default:
                break;
        }

        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(g_trajectory_task_handle, 
                            &higher_priority_task_woken);

    }
}