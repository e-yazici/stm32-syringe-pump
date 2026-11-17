/**
 * @file    as5600.c
 * @brief   AS5600 magnetic rotary encoder driver implementation.
 * @author  eyazici
 * @date    2026
 *
 * @details Reads the 12-bit raw angle from the AS5600 via I2C, applies a
 *          user-defined zero-offset correction, and tracks multi-turn
 *          rotations by detecting quadrant boundary crossings.
 *
 *          The multi-turn algorithm works by dividing the 360° circle into
 *          four quadrants and monitoring transitions:
 *          - Q4 → Q1 crossing indicates a clockwise full revolution  → totalTurns++
 *          - Q1 → Q4 crossing indicates a counter-clockwise revolution → totalTurns--
 *
 * @note    Uses I2C2 with blocking HAL calls (5 ms timeout). If the I2C bus
 *          hangs, the driver attempts a peripheral re-init as recovery.
 */

#include "as5600.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stepper.h"

extern volatile StepperControl_t gStepperControl;

/* Static atomic snapshot cache */
static AS5600_Snapshot_t s_latestSnapshot = {0};

/** @brief I2C2 handle (defined in i2c.c by CubeMX) */
extern I2C_HandleTypeDef hi2c2;

/** @brief Global encoder state — holds all raw and derived angle data */
AS5600_Params_t gEncoderParams;

/** @brief 2-byte buffer for burst read of RAW ANGLE registers (0x0C and 0x0D) */
static uint8_t gAS5600_I2C_RX_Buf[2];

/**
 * @brief  Read and decode the AS5600 magnet status register.
 *
 * @details Reads the STATUS register (0x0B) into gEncoderParams.magnetStatus.
 */
void AS5600_Check_Magnet_Status(void){
	HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, 0x0B, 1, &gEncoderParams.magnetStatus, 1, 5);
}

/**
 * @brief  Read the raw 12-bit angle and convert to degrees.
 *
 * @details Performs a 2-byte burst read starting from register 0x0C (RAW ANGLE
 *          high byte). The AS5600 auto-increments the register pointer, so both
 *          0x0C and 0x0D are read in a single I2C transaction.
 *
 *          The 12-bit raw value is assembled as:
 *            rawAngle = (rx_buf[0] & 0x0F) << 8 | rx_buf[1]
 *
 *          On I2C error, the peripheral is re-initialized as a recovery attempt.
 */
HAL_StatusTypeDef AS5600_Read_Angle(void){

	/* Burst read 2 bytes starting from RAW_ANGLE high register (0x0C) */
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, AS5600_RAW_11_8_BITS, I2C_MEMADD_SIZE_8BIT, gAS5600_I2C_RX_Buf, 2, 5);

	return status;
}

/**
 * @brief  Process the 2 bytes read from the AS5600, update quadrant and apply filtering.
 */
void AS5600_Process_I2C_Buffer(void){
	/* Retain initialization state across 100 Hz task cycles to seed zero-offset and EMA filter once */
	static uint8_t isBootInitialized = 0;
	static uint8_t filterInitialized = 0;

	gEncoderParams.rawAngleHBits = gAS5600_I2C_RX_Buf[0];
	gEncoderParams.rawAngleLBits = gAS5600_I2C_RX_Buf[1];

	uint16_t rawAngle = AS5600_Math_AssembleRawAngle(gAS5600_I2C_RX_Buf[0], gAS5600_I2C_RX_Buf[1]);

	AS5600_Math_ProcessSample(&gEncoderParams,
	                          rawAngle,
	                          &isBootInitialized,
	                          &filterInitialized,
	                          AS5600_DEFAULT_SCREW_LEAD_MM,
	                          AS5600_SAMPLE_DT_SEC);

	/* When motor is stationary (IDLE or PAUSED), snap filtered angle immediately to raw totalAngle
	 * if residual motion EMA lag (> 0.18 deg = ~2 LSB) is present, eliminating post-move settling tail
	 * while preserving normal EMA smoothing for <= 2 LSB stationary ADC jitter. */
	bool motorStationary = (gStepperControl.MotorState == MOTOR_STATE_IDLE ||
	                        gStepperControl.MotorState == MOTOR_STATE_PAUSED);
	(void)AS5600_Math_SnapIfStationary(&gEncoderParams,
	                                   motorStationary,
	                                   AS5600_STATIONARY_SNAP_DEG,
	                                   AS5600_DEFAULT_SCREW_LEAD_MM);

	/* Atomically update snapshot cache */
	taskENTER_CRITICAL();
	s_latestSnapshot.travelledDistance_mm = gEncoderParams.travelledDistance_mm;
	s_latestSnapshot.currentVelocity_mm_s = gEncoderParams.currentVelocity_mm_s;
	s_latestSnapshot.filteredTotalAngle   = gEncoderParams.filteredTotalAngle;
	s_latestSnapshot.totalTurns           = gEncoderParams.totalTurns;
	s_latestSnapshot.timestamp_ms         = (uint32_t)xTaskGetTickCount();
	s_latestSnapshot.isValid              = (isBootInitialized != 0);
	taskEXIT_CRITICAL();
}


void AS5600_I2C_Error_Handler(void){
	/* Reset I2C peripheral to clear bus-busy or arbitration-loss lockups without rebooting MCU */
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_Init(&hi2c2);

	/* Invalidate snapshot on communication error */
	taskENTER_CRITICAL();
	s_latestSnapshot.isValid = false;
	taskEXIT_CRITICAL();
}

void AS5600_GetSnapshot(AS5600_Snapshot_t *outSnapshot) {
	if (outSnapshot == NULL) {
		return;
	}
	taskENTER_CRITICAL();
	*outSnapshot = s_latestSnapshot;
	taskEXIT_CRITICAL();
}

