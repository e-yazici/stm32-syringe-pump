/**
 * @file    as5600.h
 * @brief   Driver for the AMS AS5600 magnetic rotary position sensor.
 * @author  eyazici
 * @date    2026
 *
 * @details The AS5600 is a 12-bit magnetic rotary encoder that communicates
 *          over I2C. It outputs absolute angular position (0–4095 counts = 0–360°).
 *          This driver reads raw angle data, applies a zero-offset correction,
 *          and tracks multi-turn rotations via quadrant-crossing detection.
 *
 * @note    The AS5600 has a fixed I2C address of 0x36 (7-bit). A diametrically
 *          magnetized magnet must be placed above the IC for correct operation.
 *
 * @note    Thanks to https://curiousscientist.tech/ for sharing the AS5600 application notes.
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stdint.h"
#include "main.h"
#include "as5600_math.h"

/** @defgroup AS5600_Registers AS5600 I2C Register Definitions
 *  @brief   I2C slave address and register map constants.
 *  @{
 */
#define AS5600_I2C_ADDR			(0x36 << 1)       ///< 7-bit address left-shifted for HAL I2C (write addr = 0x6C)
#define AS5600_RAW_7_0_BITS		0x0D              ///< Register: Raw Angle [7:0] (low byte)
#define AS5600_RAW_11_8_BITS	0x0C              ///< Register: Raw Angle [11:8] (high nibble, bits [3:0] valid)
/** @} */

/**
 * @brief  Global encoder state — holds all raw and derived angle data.
 */
extern AS5600_Params_t gEncoderParams;

/**
 * @brief  Read and decode the AS5600 magnet status register (0x0B).
 *
 * @details Populates gEncoderParams.magnetStatus with the raw STATUS register
 *          byte containing diagnostic bits MD (bit 5), ML (bit 4), MH (bit 3).
 *          Call this once during initialization to verify the magnet is
 *          correctly placed.
 *
 * @note   Uses blocking I2C read (5 ms timeout).
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
 * @brief  Process the 2 bytes read from I2C, update quadrant and apply filtering.
 */
void AS5600_Process_I2C_Buffer(void);

void AS5600_I2C_Error_Handler(void);

/**
 * @brief Thread-safe atomic snapshot of AS5600 encoder odometry state.
 */
typedef struct {
    float    travelledDistance_mm;   ///< Filtered linear travel distance (mm)
    float    currentVelocity_mm_s;   ///< Low-pass filtered linear velocity (mm/s)
    float    filteredTotalAngle;     ///< EMA filtered cumulative shaft angle (degrees)
    int32_t  totalTurns;             ///< Cumulative full revolutions count
    uint32_t timestamp_ms;           ///< FreeRTOS tick count at sample time (ms)
    bool     isValid;                ///< True if encoder initialized and valid sample acquired
} AS5600_Snapshot_t;

/**
 * @brief  Thread-safe retrieval of the latest encoder snapshot.
 * @param[out] outSnapshot Pointer to caller-allocated AS5600_Snapshot_t structure.
 */
void AS5600_GetSnapshot(AS5600_Snapshot_t *outSnapshot);

#endif /* INC_AS5600_H_ */
