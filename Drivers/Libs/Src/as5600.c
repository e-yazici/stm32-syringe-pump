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
 * @note    Uses I2C2 with blocking HAL calls (HAL_MAX_DELAY). If the I2C bus
 *          hangs, the driver attempts a peripheral re-init as recovery.
 */

#include "as5600.h"
#include "main.h"
#include "math.h"

/** @brief I2C1 handle (defined in i2c.c by CubeMX) */
extern I2C_HandleTypeDef hi2c2;

/** @brief Global encoder state — holds all raw and derived angle data */
AS5600_Params_t gEncoderParams;

/** @brief Buffer for DMA I2C reads */
uint8_t gAS5600_I2C_RX_Buf[2];

/** @brief EMA filter constant */
#define AS5600_EMA_ALPHA 0.2f

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

	HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, 0x0B, 1, &gEncoderParams.magnetStatus, 1, 5);

	/* Extract individual status bits from the STATUS register byte */
	magnet_detected = (gEncoderParams.magnetStatus & 0x20) >> 5; /* Bit 5: MD — 1 if magnet is present */
	magnet_too_weak = (gEncoderParams.magnetStatus & 0x10) >> 4; /* Bit 4: ML — 1 if magnet is too far */
	magnet_too_strong = (gEncoderParams.magnetStatus & 0x08) >> 3; /* Bit 3: MH — 1 if magnet is too close */
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
 * @brief  Apply zero-offset correction to the raw degree angle.
 *
 * @details Subtracts the stored startAngle (set during calibration) from
 *          the raw degreeAngle. If the result is negative (angle wrapped past
 *          0°), 360° is added to keep the value in the 0–360° range.
 */
void AS5600_Correct_Angle(void){

	gEncoderParams.correctedAngle = gEncoderParams.degreeAngle - gEncoderParams.startAngle;

	/* Wrap negative values back into 0–360° range */
	if(gEncoderParams.correctedAngle < 0){
		gEncoderParams.correctedAngle += 360;
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
	if(gEncoderParams.correctedAngle >= 0 && gEncoderParams.correctedAngle < 90){
		gEncoderParams.prevQuadrant = gEncoderParams.recentQuadrant;
		gEncoderParams.recentQuadrant = Quadrant_1;
	}
	if(gEncoderParams.correctedAngle >= 90 && gEncoderParams.correctedAngle < 180){
		gEncoderParams.prevQuadrant = gEncoderParams.recentQuadrant;
		gEncoderParams.recentQuadrant = Quadrant_2;
	}
	if(gEncoderParams.correctedAngle >= 180 && gEncoderParams.correctedAngle < 270){
		gEncoderParams.prevQuadrant = gEncoderParams.recentQuadrant;
		gEncoderParams.recentQuadrant = Quadrant_3;
	}
	if(gEncoderParams.correctedAngle >= 270 && gEncoderParams.correctedAngle < 360){
		gEncoderParams.prevQuadrant = gEncoderParams.recentQuadrant;
		gEncoderParams.recentQuadrant = Quadrant_4;
	}

	/*
	 * Detect full-revolution boundary crossings by checking for
	 * transitions between Q4 and Q1 (the 360°/0° boundary).
	 */
	if(gEncoderParams.prevQuadrant == Quadrant_4 && gEncoderParams.recentQuadrant == Quadrant_1){
		/* CW full revolution detected: Q4 → Q1 crossing (past 360°/0° boundary) */
		gEncoderParams.prevTurns = gEncoderParams.totalTurns;
		gEncoderParams.totalTurns++;
	}
	if(gEncoderParams.prevQuadrant == Quadrant_1 && gEncoderParams.recentQuadrant == Quadrant_4){
		/* CCW full revolution detected: Q1 → Q4 crossing (past 0°/360° boundary) */
		gEncoderParams.prevTurns = gEncoderParams.totalTurns;
		gEncoderParams.totalTurns--;
	}

	/* Compute cumulative absolute angle across all revolutions */
	gEncoderParams.totalAngle = (gEncoderParams.totalTurns * 360) + gEncoderParams.correctedAngle;
}


/**
 * @brief  Process the 2 bytes received via DMA, update quadrant and apply filtering.
 */
void AS5600_Process_I2C_Buffer(void){
	gEncoderParams.rawAngleHBits = gAS5600_I2C_RX_Buf[0];
	gEncoderParams.rawAngleLBits = gAS5600_I2C_RX_Buf[1];

	/* Assemble the 12-bit raw angle */
	gEncoderParams.rawAngle = ((uint16_t)(gEncoderParams.rawAngleHBits & 0x0F) << 8) | gEncoderParams.rawAngleLBits;

	/* Convert 12-bit count to degrees */
	gEncoderParams.degreeAngle = (float)gEncoderParams.rawAngle * AS5600_RAW_TO_DEGREE;

	/* Zero the encoder position on the very first read (after boot jerk) */
	static uint8_t isBootInitialized = 0;
	if (!isBootInitialized) {
		gEncoderParams.startAngle = gEncoderParams.degreeAngle;
		isBootInitialized = 1;
	}

	/* Apply zero-offset correction */
	AS5600_Correct_Angle();

	/* Check quadrant and calculate totalAngle */
	AS5600_Check_Quadrant();

	/* Initialize EMA filter on first read */
	static uint8_t filterInitialized = 0;
	if (!filterInitialized) {
		gEncoderParams.filteredTotalAngle = (float)gEncoderParams.totalAngle;
		filterInitialized = 1;
	}
	else {
		// EMA with dynamic alpha
		float error = fabs(gEncoderParams.totalAngle - gEncoderParams.filteredTotalAngle);

		float dynamic_alpha;
		if (error < 0.05f) {
		    dynamic_alpha = 1.0f;
		}
		else {
		    // Linear transition between 0.2 and 1.0
		    dynamic_alpha = 0.2f;
		}

		gEncoderParams.filteredTotalAngle = (dynamic_alpha * (float)gEncoderParams.totalAngle) +
		                                    ((1.0f - dynamic_alpha) * gEncoderParams.filteredTotalAngle);

	}

	float last_distance = gEncoderParams.travelledDistance_mm;
	gEncoderParams.travelledDistance_mm = (gEncoderParams.filteredTotalAngle / 360.0f) * 2.0f;

	/* Calculate and filter instantaneous velocity (assume 10ms update rate) */
	float raw_velocity_mm_s = (gEncoderParams.travelledDistance_mm - last_distance) / 0.01f;
	gEncoderParams.currentVelocity_mm_s = (0.1f * raw_velocity_mm_s) + (0.9f * gEncoderParams.currentVelocity_mm_s);
}


void AS5600_I2C_Error_Handler(void){
	/* Bus recovery if error/timeout */
	HAL_I2C_DeInit(&hi2c2);
	HAL_I2C_Init(&hi2c2);
}
