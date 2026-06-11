/**
 * @file    as5600.c
 * @brief   AS5600 magnetic rotary encoder driver implementation.
 * @author  Emre
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
 * @note    Uses I2C1 with blocking HAL calls (HAL_MAX_DELAY). If the I2C bus
 *          hangs, the driver attempts a peripheral re-init as recovery.
 */

#include "as5600.h"
#include "main.h"

/** @brief I2C1 handle (defined in i2c.c by CubeMX) */
extern I2C_HandleTypeDef hi2c1;

/** @brief Global encoder state — holds all raw and derived angle data */
AS5600_Params_t EncoderParams;

/** @brief Magnet detection status flags (set by AS5600_Check_Magnet_Status) */
uint8_t magnet_detected;    ///< 1 if magnet is present and within range
uint8_t magnet_too_weak;    ///< 1 if magnet field is below threshold (too far)
uint8_t magnet_too_strong;  ///< 1 if magnet field exceeds threshold (too close)


/**
 * @brief  Read and decode the AS5600 magnet status register.
 *
 * @details Reads the STATUS register (0x0B) and extracts the three
 *          magnet diagnostic bits:
 *          - Bit 5 (MD): Magnet Detected
 *          - Bit 4 (ML): Magnet Too Weak (AGC max)
 *          - Bit 3 (MH): Magnet Too Strong (AGC min)
 */
void AS5600_Check_Magnet_Status(void){

	HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, 0x0B, 1, &EncoderParams.magnetStatus, 1, HAL_MAX_DELAY);

	/* Extract individual status bits from the STATUS register byte */
	magnet_detected = (EncoderParams.magnetStatus & 0x20) >> 5; /* Bit 5: MD — 1 if magnet is present */
	magnet_too_weak = (EncoderParams.magnetStatus & 0x10) >> 4; /* Bit 4: ML — 1 if magnet is too far */
	magnet_too_strong = (EncoderParams.magnetStatus & 0x08) >> 3; /* Bit 3: MH — 1 if magnet is too close */
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
void AS5600_Read_Angle(void){

	uint8_t rx_buf[2];

	/* Burst read 2 bytes starting from RAW_ANGLE high register (0x0C) */
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDR, AS5600_RAW_11_8_BITS, I2C_MEMADD_SIZE_8BIT, rx_buf, 2, HAL_MAX_DELAY);
	if (status == HAL_OK) {
		EncoderParams.rawAngleHBits = rx_buf[0]; /* High nibble from register 0x0C */
		EncoderParams.rawAngleLBits = rx_buf[1]; /* Low byte from register 0x0D */

		/* Assemble the 12-bit raw angle: mask off upper 4 bits of high byte */
		EncoderParams.rawAngle = ((uint16_t)(EncoderParams.rawAngleHBits & 0x0F) << 8) | EncoderParams.rawAngleLBits;

		/* Convert 12-bit count (0–4095) to degrees (0.0–360.0) */
		EncoderParams.degreeAngle = (float)EncoderParams.rawAngle * AS5600_RAW_TO_DEGREE;
	}
	else{
		/* I2C error recovery: attempt to re-initialize the peripheral */
		HAL_I2C_Init(&hi2c1);
	}

}

/**
 * @brief  Apply zero-offset correction to the raw degree angle.
 *
 * @details Subtracts the stored startAngle (set during calibration) from
 *          the raw degreeAngle. If the result is negative (angle wrapped past
 *          0°), 360° is added to keep the value in the 0–360° range.
 */
void AS5600_Correct_Angle(void){

	EncoderParams.correctedAngle = EncoderParams.degreeAngle - EncoderParams.startAngle;

	/* Wrap negative values back into 0–360° range */
	if(EncoderParams.correctedAngle < 0){
		EncoderParams.correctedAngle += 360;
	}
	else{
		__NOP(); /* No correction needed */
	}
}

/**
 * @brief  Detect quadrant transitions and update the multi-turn counter.
 *
 * @details Determines the current quadrant (1–4) based on correctedAngle,
 *          then checks for full-revolution boundary crossings:
 *
 *          - **CW revolution**:  prevQuadrant == Q4 && recentQuadrant == Q1
 *            → totalTurns++ (shaft completed one clockwise revolution)
 *          - **CCW revolution**: prevQuadrant == Q1 && recentQuadrant == Q4
 *            → totalTurns-- (shaft completed one counter-clockwise revolution)
 *
 *          Finally computes: totalAngle = (totalTurns * 360) + correctedAngle
 *
 * @note   This function must be called after AS5600_Correct_Angle() on every
 *         polling cycle for reliable turn counting.
 */
void AS5600_Check_Quadrant(void){
	/* Determine which 90° quadrant the corrected angle falls in */
	if(EncoderParams.correctedAngle >= 0 && EncoderParams.correctedAngle < 90){
		EncoderParams.prevQuadrant = EncoderParams.recentQuadrant;
		EncoderParams.recentQuadrant = Quadrant_1;
	}
	if(EncoderParams.correctedAngle >= 90 && EncoderParams.correctedAngle < 180){
		EncoderParams.prevQuadrant = EncoderParams.recentQuadrant;
		EncoderParams.recentQuadrant = Quadrant_2;
	}
	if(EncoderParams.correctedAngle >= 180 && EncoderParams.correctedAngle < 270){
		EncoderParams.prevQuadrant = EncoderParams.recentQuadrant;
		EncoderParams.recentQuadrant = Quadrant_3;
	}
	if(EncoderParams.correctedAngle >= 270 && EncoderParams.correctedAngle < 360){
		EncoderParams.prevQuadrant = EncoderParams.recentQuadrant;
		EncoderParams.recentQuadrant = Quadrant_4;
	}

	/*
	 * Detect full-revolution boundary crossings by checking for
	 * transitions between Q4 and Q1 (the 360°/0° boundary).
	 */
	if(EncoderParams.prevQuadrant == Quadrant_4 && EncoderParams.recentQuadrant == Quadrant_1){
		/* CW full revolution detected: Q4 → Q1 crossing (past 360°/0° boundary) */
		EncoderParams.prevTurns = EncoderParams.totalTurns;
		EncoderParams.totalTurns++;
	}
	if(EncoderParams.prevQuadrant == Quadrant_1 && EncoderParams.recentQuadrant == Quadrant_4){
		/* CCW full revolution detected: Q1 → Q4 crossing (past 0°/360° boundary) */
		EncoderParams.prevTurns = EncoderParams.totalTurns;
		EncoderParams.totalTurns--;
	}

	/* Compute cumulative absolute angle across all revolutions */
	EncoderParams.totalAngle = (EncoderParams.totalTurns * 360) + EncoderParams.correctedAngle;
}
