/**
 * @file    as5600.h
 * @brief   Driver for the AMS AS5600 magnetic rotary position sensor.
 * @author  Emre
 * @date    2026
 *
 * @details The AS5600 is a 12-bit magnetic rotary encoder that communicates
 *          over I2C. It outputs absolute angular position (0–4095 counts = 0–360°).
 *          This driver reads raw angle data, applies a zero-offset correction,
 *          and tracks multi-turn rotations via quadrant-crossing detection.
 *
 * @note    The AS5600 has a fixed I2C address of 0x36 (7-bit). A diametrically
 *          magnetized magnet must be placed above the IC for correct operation.
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stdint.h"
#include "main.h"

/** @defgroup AS5600_Registers AS5600 I2C Register Definitions
 *  @brief   I2C slave address and register map constants.
 *  @{
 */
#define AS5600_I2C_ADDR			(0x36 << 1)       ///< 7-bit address left-shifted for HAL I2C (write addr = 0x6C)
#define AS5600_RAW_7_0_BITS		0x0D              ///< Register: Raw Angle [7:0] (low byte)
#define AS5600_RAW_11_8_BITS	0x0C              ///< Register: Raw Angle [11:8] (high nibble, bits [3:0] valid)
#define AS5600_RAW_TO_DEGREE	0.087890625f      ///< Conversion factor: raw_count * (360.0/4096.0) = degrees
/** @} */

/**
 * @brief  Quadrant identifiers for multi-turn tracking.
 *
 * @details The 360° circle is divided into four quadrants (Q1–Q4). By detecting
 *          transitions between Q4→Q1 (CW full turn) and Q1→Q4 (CCW full turn),
 *          the driver accumulates a total turn count.
 */
typedef enum{
	Quadrant_1 = 1,  ///< 0°–89°
	Quadrant_2,      ///< 90°–179°
	Quadrant_3,      ///< 180°–269°
	Quadrant_4       ///< 270°–359°
}AS5600_Quadrant_e;

/**
 * @brief  Runtime parameter structure holding all encoder state.
 *
 * @details This structure is maintained by the AS5600_* functions and contains
 *          both raw I2C data and derived values (corrected angle, turn count).
 */
typedef struct{
	uint8_t magnetStatus;              ///< Raw STATUS register (0x0B): bits [5]=MD, [4]=ML, [3]=MH
	float startAngle;                  ///< Zero-offset angle in degrees, set at calibration
	uint16_t rawAngle;                 ///< Combined 12-bit raw angle (0–4095)
	uint8_t rawAngleLBits;             ///< Raw register byte from 0x0D (bits [7:0])
	uint8_t rawAngleHBits;             ///< Raw register byte from 0x0C (bits [3:0] valid)
	float degreeAngle;                 ///< Raw angle converted to degrees (0.0–360.0)
	float correctedAngle;              ///< Angle after zero-offset subtraction (0.0–360.0)
	int32_t totalAngle;                ///< Cumulative angle in degrees (turns * 360 + corrected)
	int32_t totalTurns;                ///< Number of complete revolutions (increments/decrements on Q4↔Q1 crossing)
	int32_t prevTurns;                 ///< Previous turn count (for edge-detection logic)
	AS5600_Quadrant_e recentQuadrant;  ///< Current quadrant based on correctedAngle
	AS5600_Quadrant_e prevQuadrant;    ///< Previous quadrant (used for crossing detection)
	float filteredTotalAngle;          ///< EMA filtered total angle
	float travelledDistance_mm;        ///< Distance calculated from the filtered angle
	float currentVelocity_mm_s;
}AS5600_Params_t;

/**
 * @brief  Read and decode the AS5600 magnet status register (0x0B).
 *
 * @details Populates EncoderParams.magnetStatus and sets the module-level
 *          flags: magnet_detected, magnet_too_weak, magnet_too_strong.
 *          Call this once during initialization to verify the magnet is
 *          correctly placed.
 *
 * @note   Uses blocking I2C read (HAL_MAX_DELAY).
 */
void AS5600_Check_Magnet_Status(void);

/**
 * @brief  Read the raw 12-bit angle from the AS5600
 *
 * @details Performs a 2-byte I2C read from registers 0x0C–0x0D, assembles the
 *          12-bit value
 * @return HAL_StatusTypeDef (HAL_OK if started successfully)
 */
HAL_StatusTypeDef AS5600_Read_Angle(void);

/**
 * @brief  Apply zero-offset correction to the raw degree angle.
 *
 * @details Subtracts the stored startAngle from degreeAngle and wraps the
 *          result into the 0–360° range. The corrected value is stored in
 *          EncoderParams.correctedAngle.
 */
void AS5600_Correct_Angle(void);

/**
 * @brief  Detect quadrant transitions and update the multi-turn counter.
 *
 * @details Determines which quadrant (1–4) the correctedAngle falls in, then
 *          checks for full-revolution crossings:
 *          - Q4 → Q1 transition: increment totalTurns (CW revolution)
 *          - Q1 → Q4 transition: decrement totalTurns (CCW revolution)
 *          Updates totalAngle = totalTurns * 360 + correctedAngle.
 */
void AS5600_Check_Quadrant(void);

/**
 * @brief  Process the 2 bytes received via DMA, update quadrant and apply filtering.
 */
void AS5600_Process_I2C_Buffer(void);

void AS5600_I2C_Error_Handler(void);

#endif /* INC_AS5600_H_ */
