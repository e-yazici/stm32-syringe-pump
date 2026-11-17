/**
 * @file    stepper.c
 * @brief   Stepper motor S-curve acceleration engine implementation.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements the stepper motor acceleration profile generator and
 *          motion start sequence. The acceleration profile uses a cosine
 *          easing function to produce a smooth S-shaped velocity ramp, which
 *          reduces mechanical vibration, motor stall and improves accuracy at high speeds.
 *
 *          The motion is executed as a trapezoidal profile with three phases:
 *          1. **Acceleration**: Step frequency increases from START_FREQ_HZ
 *             to cruiseFreqHz following the S-curve lookup table.
 *          2. **Cruise**: Constant frequency at cruiseFreqHz.
 *          3. **Deceleration**: Mirror of acceleration (table read in reverse).
 *
 *          If the total move is too short for a full trapezoid, the cruise
 *          phase is eliminated resulting in a triangular profile.
 *
 * @note    The actual step-by-step state machine execution happens in the
 *          HAL_TIM_PeriodElapsedCallback() ISR in main.c. This module only
 *          sets up the profile and starts the timer.
 */

#include "stepper.h"
#include "usart.h"
#include "rtos_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <limits.h>
#include <string.h>

/*
 * Expand the IC cache to support all 4 possible UART addresses (0-3).
 * The TMC2208 UART address is set by the MS1/MS2 pins: if they are not
 * explicitly tied to GND the address may not be 0.  With IC_CACHE_COUNT=4
 * shadow registers work correctly regardless of which address is used.
 * Must be defined BEFORE including TMC2208.h because of the #ifndef guard.
 */
#ifndef TMC2208_IC_CACHE_COUNT
#define TMC2208_IC_CACHE_COUNT 4
#endif

#include "TMC2208.h"

#define TMC_EXPECTED_MRES       1U          /* 128 microsteps */
#define TMC_CHOPCONF_DEFAULT    0x10000053U
#define TMC_PWMCONF_DEFAULT     0xC10D0024U
#define TMC_GCONF_DEFAULT       0x00000041U
#define TMC_MAX_COMM_RETRIES    5U

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TaskHandle_t TaskUartGatekeeperHandle;

static uint16_t node_id = 0xFFFFU;
static uint16_t s_last_valid_node_id = 0xFFFFU;
static uint8_t s_spreadcycle_enabled = 0U;
static TickType_t s_last_verified_tick = 0U;
static bool s_config_verified = false;
float s_curve_norm[256];

typedef enum {
    UART_REQ_TMC2208_TXRX = 0,
    UART_REQ_VCP_TX
} UartReqType_e;

#define UART_VCP_MAX_PAYLOAD 64U

typedef struct {
    UartReqType_e type;
    uint8_t      *txRxData;      /* Used by UART_REQ_TMC2208_TXRX */
    uint16_t      writeLength;
    uint16_t      readLength;
    TaskHandle_t  callerTask;    /* Task to notify upon synchronous completion */
    bool         *outSuccess;    /* Pointer to caller's bool result */
    uint8_t       vcpPayload[UART_VCP_MAX_PAYLOAD]; /* Used by async UART_REQ_VCP_TX */
} UartGatekeeperReq_t;

typedef enum {
    U1_PHASE_IDLE = 0,
    U1_PHASE_TX,
    U1_PHASE_RX
} Uart1Phase_e;

#define UART_NOTIFY_OK   (1UL << 0)
#define UART_NOTIFY_ERR  (1UL << 1)

static QueueHandle_t sUartGatekeeperQueue = NULL;
static uint8_t s_u1_tx_buf[16];
static uint8_t s_u1_rx_buf[16];
static uint8_t s_u2_dma_buf[UART_VCP_MAX_PAYLOAD];
static volatile Uart1Phase_e s_u1_phase = U1_PHASE_IDLE;

void stepper_uart_gatekeeper_init(void)
{
    if (sUartGatekeeperQueue == NULL) {
        sUartGatekeeperQueue = xQueueCreate(8, sizeof(UartGatekeeperReq_t));
    }
}

static void stepper_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        TickType_t ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0U ? ticks : 1U);
    } else {
        HAL_Delay(ms);
    }
}

static void stepper_reinit_uart(void)
{
    (void)HAL_UART_Abort(&huart1);
    (void)HAL_UART_DeInit(&huart1);
    MX_USART1_UART_Init();
    stepper_delay_ms(5U);
}

static bool stepper_exec_tmc_dma(uint8_t *data, size_t writeLength, size_t readLength)
{
    size_t totalRx = writeLength + readLength;
    if (data == NULL || writeLength == 0U ||
        writeLength > sizeof(s_u1_tx_buf) || totalRx > sizeof(s_u1_rx_buf)) {
        return false;
    }

    (void)memcpy(s_u1_tx_buf, data, writeLength);
    s_u1_phase = U1_PHASE_RX;

    /* Clear any stale task notification on TaskUartGatekeeperHandle */
    (void)xTaskNotifyWait(0U, ULONG_MAX, NULL, 0U);

    /* Ensure both TE and RE remain continuously enabled in Half-Duplex mode (never toggle TE/RE) */
    if ((huart1.Instance->CR1 & (USART_CR1_TE | USART_CR1_RE)) != (USART_CR1_TE | USART_CR1_RE)) {
        ATOMIC_SET_BIT(huart1.Instance->CR1, (USART_CR1_TE | USART_CR1_RE));
    }

    /* Flush RX data register and clear stale UART error flags */
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                    UART_CLEAR_PEF | UART_CLEAR_FEF);

    /* Arm RX DMA first for (writeLength echo bytes + readLength reply bytes) */
    if (HAL_UART_Receive_DMA(&huart1, s_u1_rx_buf, (uint16_t)totalRx) != HAL_OK) {
        (void)HAL_UART_Abort(&huart1);
        s_u1_phase = U1_PHASE_IDLE;
        return false;
    }
    /* Disable Error Interrupt (EIE) so single-wire noise flags never abort DMA; CRC8 validates payload */
    ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);

    /* Start TX DMA */
    if (HAL_UART_Transmit_DMA(&huart1, s_u1_tx_buf, (uint16_t)writeLength) != HAL_OK) {
        (void)HAL_UART_Abort(&huart1);
        s_u1_phase = U1_PHASE_IDLE;
        return false;
    }

    uint32_t notifyVal = 0U;
    BaseType_t ok = xTaskNotifyWait(0U, ULONG_MAX, &notifyVal, pdMS_TO_TICKS(20U));

    if (ok != pdTRUE || (notifyVal & UART_NOTIFY_OK) == 0U) {
        (void)HAL_UART_Abort(&huart1);
        s_u1_phase = U1_PHASE_IDLE;
        s_config_verified = false;
        vTaskDelay(pdMS_TO_TICKS(2U));
        return false;
    }

    if (readLength > 0U) {
        (void)memcpy(data, &s_u1_rx_buf[writeLength], readLength);
    }

    /* Non-blocking bus turnaround sleep (2 ticks = 1.0 to 2.0 ms bus-idle time = 38-76 bit times at 38,400 baud) */
    vTaskDelay(pdMS_TO_TICKS(2U));
    return true;
}

bool stepper_verify_tmc_config(void)
{
    if (node_id >= 4U) {
        s_config_verified = false;
        return false;
    }

    int32_t gconf_raw = 0;
    if (!tmc2208_readRegisterChecked(node_id, TMC2208_GCONF, &gconf_raw)) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)gconf_raw, TMC2208_PDN_DISABLE_FIELD) != 1U) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)gconf_raw, TMC2208_MSTEP_REG_SELECT_FIELD) != 1U) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)gconf_raw, TMC2208_MULTISTEP_FILT_FIELD) != 0U) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)gconf_raw, TMC2208_EN_SPREADCYCLE_FIELD) != (s_spreadcycle_enabled ? 1U : 0U)) {
        s_config_verified = false;
        return false;
    }

    int32_t chopconf_raw = 0;
    if (!tmc2208_readRegisterChecked(node_id, TMC2208_CHOPCONF, &chopconf_raw)) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)chopconf_raw, TMC2208_MRES_FIELD) != TMC_EXPECTED_MRES) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)chopconf_raw, TMC2208_INTPOL_FIELD) != 0U) {
        s_config_verified = false;
        return false;
    }
    if (tmc2208_fieldExtract((uint32_t)chopconf_raw, TMC2208_TOFF_FIELD) == 0U) {
        s_config_verified = false;
        return false;
    }

    s_config_verified = true;
    s_last_verified_tick = HAL_GetTick();
    return true;
}

bool stepper_recover_tmc(void)
{
    s_config_verified = false;

    for (uint32_t attempt = 0U; attempt < TMC_MAX_COMM_RETRIES; attempt++) {
        int32_t ioin_raw = 0;

        /* On attempt > 0 or if address is unknown, reinit UART and scan for node */
        if (attempt > 0U || node_id >= 4U) {
            if (attempt > 0U) {
                stepper_reinit_uart();
            }
            node_id = 0xFFFFU;
            for (uint16_t addr = 0U; addr < 4U; addr++) {
                if (tmc2208_readRegisterChecked(addr, TMC2208_IOIN, &ioin_raw)) {
                    if ((((uint32_t)ioin_raw >> 24) & 0xFFU) == 0x20U) {
                        node_id = addr;
                        s_last_valid_node_id = addr;
                        break;
                    }
                }
            }
            if (node_id >= 4U) {
                stepper_delay_ms(10U);
                continue;
            }
        } else {
            s_last_valid_node_id = node_id;
        }

        /* Pre-seed shadow cache with hardware defaults */
        uint32_t chopconf_default = TMC_CHOPCONF_DEFAULT;
        uint32_t pwmconf_default  = TMC_PWMCONF_DEFAULT;
        (void)tmc2208_cache(node_id, TMC2208_CACHE_FILL_DEFAULT, TMC2208_CHOPCONF, &chopconf_default);
        (void)tmc2208_cache(node_id, TMC2208_CACHE_FILL_DEFAULT, TMC2208_PWMCONF,  &pwmconf_default);

        /* Clear TMC2208_GSTAT by writing 0x07 (RESET | DRV_ERR | UV_CP) */
        tmc2208_writeRegister(node_id, TMC2208_GSTAT, 0x07);

        /* Read TMC2208_GCONF; fallback to default if read fails */
        int32_t gconf_raw = 0;
        uint32_t gconf = TMC_GCONF_DEFAULT;
        if (tmc2208_readRegisterChecked(node_id, TMC2208_GCONF, &gconf_raw)) {
            gconf = (uint32_t)gconf_raw;
        }
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_PDN_DISABLE_FIELD,      1U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_MSTEP_REG_SELECT_FIELD, 1U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_EN_SPREADCYCLE_FIELD,   s_spreadcycle_enabled ? 1U : 0U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_MULTISTEP_FILT_FIELD,   0U);
        tmc2208_writeRegister(node_id, TMC2208_GCONF, (int32_t)gconf);

        /* Read TMC2208_CHOPCONF; fallback to default if read fails or TOFF == 0 */
        int32_t chopconf_raw = 0;
        uint32_t chopconf = TMC_CHOPCONF_DEFAULT;
        if (tmc2208_readRegisterChecked(node_id, TMC2208_CHOPCONF, &chopconf_raw)) {
            if (tmc2208_fieldExtract((uint32_t)chopconf_raw, TMC2208_TOFF_FIELD) != 0U) {
                chopconf = (uint32_t)chopconf_raw;
            }
        }
        chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_MRES_FIELD,   TMC_EXPECTED_MRES);
        chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_INTPOL_FIELD, 0U);
        tmc2208_writeRegister(node_id, TMC2208_CHOPCONF, (int32_t)chopconf);

        /* Read IFCNT before and after writing TPWMTHRS = 0 to verify write datagram receipt */
        int32_t ifcnt_before_raw = 0;
        int32_t ifcnt_after_raw = 0;
        bool ifcnt_verified = false;
        if (tmc2208_readRegisterChecked(node_id, TMC2208_IFCNT, &ifcnt_before_raw)) {
            tmc2208_writeRegister(node_id, TMC2208_TPWMTHRS, 0);
            if (tmc2208_readRegisterChecked(node_id, TMC2208_IFCNT, &ifcnt_after_raw)) {
                uint8_t diff = (uint8_t)((uint32_t)ifcnt_after_raw - (uint32_t)ifcnt_before_raw);
                if (diff >= 1U) {
                    ifcnt_verified = true;
                }
            }
        } else {
            tmc2208_writeRegister(node_id, TMC2208_TPWMTHRS, 0);
        }

        /* Call stepper_verify_tmc_config(). If verified, return true */
        if (ifcnt_verified && stepper_verify_tmc_config()) {
            s_last_valid_node_id = node_id;
            return true;
        }

        /* If verification failed, invalidate node_id for full reinit/rescan on next retry */
        node_id = 0xFFFFU;
        stepper_delay_ms(10U);
    }

    if (s_last_valid_node_id < 4U) {
        node_id = s_last_valid_node_id;
    }

    return false;
}

bool stepper_ensure_tmc_configured(void)
{
    if (s_config_verified && ((HAL_GetTick() - s_last_verified_tick) < 50U)) {
        return true;
    }
    if (stepper_verify_tmc_config()) {
        return true;
    }
    return stepper_recover_tmc();
}

/**
 * @brief  Initialize all stepper control fields to safe defaults and pre-compute
 *         the S-curve lookup table (TMC2208 UART discovery/config runs in TaskUartGatekeeper).
 * @param  stepper Pointer to the StepperControl_t instance to initialize.
 */
void stepper_init(StepperControl_t *stepper)
{
    stepper->ARRCurrent = 0;

    stepper->totalStepsRemaining = 0;
    stepper->accelSteps = 0;
    stepper->decelStartStep = 0;
    stepper->rampStepCounter = 0;

    stepper->MotorState    = MOTOR_STATE_IDLE;
    stepper->MotorMode     = MOTOR_MODE_NONE;
    
    /* Pre-compute normalized S-curve table (0.0 to 1.0) for interpolation */
    StepperProfile_GenerateSCurveTable(s_curve_norm);

    s_spreadcycle_enabled = 0U;
    s_config_verified = false;
    s_last_verified_tick = 0U;
}

/**
 * @brief  Switch the TMC2208 chopper mode at runtime.
 *
 * @param  enable  true  → SpreadCycle (use when motor is actively driving load)
 *                 false → StealthChop (use at idle — silent, lower temperature)
 *
 * @details Does a live GCONF write over UART and verifies driver configuration.
 *          Safe to call from any thread context.
 */
void stepper_set_spreadcycle(uint8_t enable)
{
    s_spreadcycle_enabled = (enable != 0U) ? 1U : 0U;

    bool verified = false;

    if (node_id >= 4U && s_last_valid_node_id < 4U) {
        node_id = s_last_valid_node_id;
    }

    if (node_id < 4U) {
        uint32_t gconf = TMC_GCONF_DEFAULT;
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_PDN_DISABLE_FIELD,      1U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_MSTEP_REG_SELECT_FIELD, 1U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_MULTISTEP_FILT_FIELD,   0U);
        gconf = tmc2208_fieldUpdate(gconf, TMC2208_EN_SPREADCYCLE_FIELD,   s_spreadcycle_enabled);
        tmc2208_writeRegister(node_id, TMC2208_GCONF, (int32_t)gconf);

        if (stepper_verify_tmc_config()) {
            verified = true;
        }
    }

    if (!verified) {
        (void)stepper_recover_tmc();
    }
}

uint8_t tmc2208_getNodeAddress(uint16_t icID)
{
    // icID directly encodes the UART node address (0–3), set by MS1/MS2 pins.
    return (uint8_t)icID;
}

bool tmc2208_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    (void)icID;
    if (data == NULL || writeLength == 0U) {
        return false;
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return false;
    }

    if (xTaskGetCurrentTaskHandle() == TaskUartGatekeeperHandle) {
        return stepper_exec_tmc_dma(data, writeLength, readLength);
    }

    UartGatekeeperReq_t req;
    bool result = false;

    req.type = UART_REQ_TMC2208_TXRX;
    req.txRxData = data;
    req.writeLength = (uint16_t)writeLength;
    req.readLength = (uint16_t)readLength;
    req.callerTask = xTaskGetCurrentTaskHandle();
    req.outSuccess = &result;

    /* Clear caller's notification state */
    (void)ulTaskNotifyTake(pdTRUE, 0U);

    if (sUartGatekeeperQueue == NULL || xQueueSend(sUartGatekeeperQueue, &req, pdMS_TO_TICKS(100U)) != pdTRUE) {
        s_config_verified = false;
        return false;
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100U)) == 0U) {
        s_config_verified = false;
        return false;
    }

    return result;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* USART1 TMC2208 completion is signaled by HAL_UART_RxCpltCallback after totalRx bytes */
        (void)huart;
    } else if (huart->Instance == USART2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (TaskUartGatekeeperHandle != NULL) {
            xTaskNotifyFromISR(TaskUartGatekeeperHandle, UART_NOTIFY_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        s_u1_phase = U1_PHASE_IDLE;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (TaskUartGatekeeperHandle != NULL) {
            xTaskNotifyFromISR(TaskUartGatekeeperHandle, UART_NOTIFY_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        if (s_u1_phase != U1_PHASE_IDLE) {
            s_u1_phase = U1_PHASE_IDLE;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (TaskUartGatekeeperHandle != NULL) {
                xTaskNotifyFromISR(TaskUartGatekeeperHandle, UART_NOTIFY_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    } else if (huart->Instance == USART2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (TaskUartGatekeeperHandle != NULL) {
            xTaskNotifyFromISR(TaskUartGatekeeperHandle, UART_NOTIFY_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void RtosTasks_TaskUartGatekeeper(void *argument)
{
    (void)argument;

    /* Defensive guard: ensure gatekeeper queue exists */
    stepper_uart_gatekeeper_init();

    /* Startup: Discover & configure TMC2208 over DMA */
    while (!stepper_recover_tmc()) {
        vTaskDelay(pdMS_TO_TICKS(20U));
    }

    static UartGatekeeperReq_t req;

    for (;;) {
        if (xQueueReceive(sUartGatekeeperQueue, &req, portMAX_DELAY) == pdTRUE) {
            if (req.type == UART_REQ_TMC2208_TXRX) {
                bool ok = stepper_exec_tmc_dma(req.txRxData, req.writeLength, req.readLength);
                if (req.outSuccess != NULL) {
                    *req.outSuccess = ok;
                }
                if (req.callerTask != NULL) {
                    xTaskNotifyGive(req.callerTask);
                }
            } else if (req.type == UART_REQ_VCP_TX) {
                if (req.writeLength > 0U && req.writeLength <= UART_VCP_MAX_PAYLOAD) {
                    (void)memcpy(s_u2_dma_buf, req.vcpPayload, req.writeLength);
                    (void)xTaskNotifyWait(0U, ULONG_MAX, NULL, 0U);
                    if (HAL_UART_Transmit_DMA(&huart2, s_u2_dma_buf, req.writeLength) == HAL_OK) {
                        uint32_t notifyVal = 0U;
                        BaseType_t ok = xTaskNotifyWait(0U, ULONG_MAX, &notifyVal, pdMS_TO_TICKS(50U));
                        if (ok != pdTRUE || (notifyVal & UART_NOTIFY_OK) == 0U) {
                            (void)HAL_UART_Abort(&huart2);
                        }
                    } else {
                        (void)HAL_UART_Abort(&huart2);
                    }
                }
            }
        }
    }
}

bool RtosTasks_UartSendVcpAsync(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || sUartGatekeeperQueue == NULL) {
        return false;
    }

    if (len > UART_VCP_MAX_PAYLOAD) {
        len = UART_VCP_MAX_PAYLOAD;
    }

    UartGatekeeperReq_t req;
    (void)memset(&req, 0, sizeof(req));
    req.type = UART_REQ_VCP_TX;
    req.writeLength = len;
    (void)memcpy(req.vcpPayload, data, len);

    return (xQueueSend(sUartGatekeeperQueue, &req, 0U) == pdTRUE);
}


/**
 * @brief  Compute the motion profile and start the stepper motor.
 * @param  stepper      Pointer to the StepperControl_t instance.
 * @param  cruiseFreqHz Desired cruise step frequency (Hz).
 * @param  totalSteps   Total steps for the move.
 * @param  accelSteps   Desired number of acceleration ramp steps.
 *
 * @details Sequence of operations:
 *          1. Reject if motor is already running.
 *          2. Verify TMC2208 UART configuration via stepper_ensure_tmc_configured().
 *          3. Stop any prior timer activity (defensive).
 *          4. Prepare ramp parameters and clamp accelSteps to totalSteps/2 for short moves.
 *          5. Configure TIM2: prescaler, initial ARR, 50% duty compare value.
 *          6. Force an update event (UG bit) to load shadow registers immediately.
 *          7. Arm the state machine to MOTOR_STATE_ACCEL.
 *          8. Start TIM2 in interrupt + PWM mode.
 */
void stepper_start(StepperControl_t *stepper, float cruiseFreqHz,
        uint32_t totalSteps, uint32_t accelSteps)
{
    /* Reject if a move is already in progress */
    if (stepper->MotorState != MOTOR_STATE_IDLE) {
        return;
    }

    /* Ensure TMC2208 configuration is verified (MS128) before starting pulses */
    (void)stepper_ensure_tmc_configured();

    /* --- Configure Timer Before Arming State Machine --- */

    /* Defensive stop: ensure timer is fully halted before reconfiguring */
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    /* Compute ramp parameters, initial ARR, and arm state machine */
    StepperProfile_PrepareMove(stepper, cruiseFreqHz, (float)START_FREQ_HZ,
                               totalSteps, accelSteps,
                               TIMER_CLOCK, PRESCALER, stepper->MotorMode);

    /* Set the prescaler (effective tick = TIMER_CLOCK / (PSC + 1) = 1 MHz) */
    __HAL_TIM_SET_PRESCALER(&htim2, PRESCALER);

    /* Load the initial ARR value (slowest speed) and 50% duty cycle */
    __HAL_TIM_SET_AUTORELOAD(&htim2, stepper->ARRCurrent);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, stepper->ARRCurrent / 2);

    /*
     * Force an Update Generation event (UG bit in EGR register).
     * This immediately loads the new PSC and ARR values from the shadow
     * registers into the active registers, so the very first period is correct.
     */
    TIM2->EGR |= TIM_EGR_UG;
    
    /* 
     * Clear pending update flag (UIF) to prevent an immediate spurious
     * step interrupt upon calling HAL_TIM_Base_Start_IT, consuming the first step prematurely.
     */
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

    /* ---- Start the Hardware ---- */
    /* Start base timer interrupt (triggers HAL_TIM_PeriodElapsedCallback each period) */
    HAL_TIM_Base_Start_IT(&htim2);
    /* Start PWM output on Channel 1 (drives the stepper STEP pin) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
