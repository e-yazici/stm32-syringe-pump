/**
 * @file    as5600_math.h
 * @brief   Hardware-agnostic AS5600 angle odometry, multi-turn tracking, and EMA filter.
 * @author  eyazici
 * @date    2026
 *
 * @details Converts 12-bit raw encoder counts to degrees, applies a zero-offset
 *          correction, tracks multi-turn rotations via quadrant-crossing detection
 *          (Q4 -> Q1 = CW turn, Q1 -> Q4 = CCW turn), and filters angle jitter
 *          using a dynamic-alpha Exponential Moving Average (EMA).
 *
 * @note    Thanks to https://curiousscientist.tech/ for sharing the AS5600 application notes.
 */

#ifndef LIBS_INC_AS5600_MATH_H_
#define LIBS_INC_AS5600_MATH_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_RAW_MAX               4096.0f
#define AS5600_RAW_TO_DEGREE         0.087890625f      ///< Conversion factor: raw_count * (360.0/4096.0) = degrees
#define AS5600_EMA_ALPHA             0.2f              ///< Position EMA filter constant
#define AS5600_VEL_EMA_ALPHA         0.33f             ///< Velocity EMA filter constant (fast tracking, ~25 ms decay)
#define AS5600_DEFAULT_SCREW_LEAD_MM  2.0f              ///< Default lead screw pitch in mm/rev
#define AS5600_SAMPLE_DT_SEC          0.01f             ///< Default sampling interval in seconds (100 Hz)
#define AS5600_DYNAMIC_EMA_THRESH_DEG 0.05f             ///< Error threshold below which position EMA alpha snaps to 1.0
#define AS5600_STATIONARY_SNAP_DEG    0.18f             ///< Stationary snap threshold in degrees (~2 LSB)

/**
 * @brief  Quadrant identifiers for multi-turn tracking.
 *
 * @details The 360° circle is divided into four quadrants (Q1–Q4). By detecting
 *          transitions between Q4→Q1 (CW full turn) and Q1→Q4 (CCW full turn),
 *          the driver accumulates a total turn count.
 */
typedef enum {
    Quadrant_1 = 1,  ///< 0°–89°
    Quadrant_2,      ///< 90°–179°
    Quadrant_3,      ///< 180°–269°
    Quadrant_4       ///< 270°–359°
} AS5600_Quadrant_e;

#define AS5600_QUADRANT_1 Quadrant_1
#define AS5600_QUADRANT_2 Quadrant_2
#define AS5600_QUADRANT_3 Quadrant_3
#define AS5600_QUADRANT_4 Quadrant_4

/**
 * @brief  Runtime parameter structure holding all encoder state.
 *
 * @details This structure is maintained by the AS5600_* functions and contains
 *          both raw I2C data and derived values (corrected angle, turn count).
 */
typedef struct {
    uint8_t magnetStatus;              ///< Raw STATUS register (0x0B): bits [5]=MD, [4]=ML, [3]=MH
    float startAngle;                  ///< Zero-offset angle in degrees, set at calibration
    uint16_t rawAngle;                 ///< Combined 12-bit raw angle (0–4095)
    uint8_t rawAngleLBits;             ///< Raw register byte from 0x0D (bits [7:0])
    uint8_t rawAngleHBits;             ///< Raw register byte from 0x0C (bits [3:0] valid)
    float degreeAngle;                 ///< Raw angle converted to degrees (0.0–360.0)
    float correctedAngle;              ///< Angle after zero-offset subtraction (0.0–360.0)
    float totalAngle;                  ///< Cumulative angle in degrees (turns * 360.0f + corrected)
    int32_t totalTurns;                ///< Number of complete revolutions (increments/decrements on Q4↔Q1 crossing)
    int32_t prevTurns;                 ///< Previous turn count (for edge-detection logic)
    AS5600_Quadrant_e recentQuadrant;  ///< Current quadrant based on correctedAngle
    AS5600_Quadrant_e prevQuadrant;    ///< Previous quadrant (used for crossing detection)
    float filteredTotalAngle;          ///< EMA filtered total angle
    float travelledDistance_mm;        ///< Distance calculated from the filtered angle
    float currentVelocity_mm_s;
} AS5600_Params_t;

/**
 * @brief  Initialize an AS5600_Params_t structure to zero/default state.
 * @param  params Pointer to AS5600_Params_t.
 */
static inline void AS5600_Math_InitParams(AS5600_Params_t *params) {
    if (params == NULL) {
        return;
    }
    memset(params, 0, sizeof(AS5600_Params_t));
}

/**
 * @brief  Assemble 12-bit raw angle from high and low register bytes.
 * @param  highByte Register 0x0C (bits [3:0] valid).
 * @param  lowByte  Register 0x0D (bits [7:0] valid).
 * @retval 12-bit raw angle (0 to 4095).
 */
static inline uint16_t AS5600_Math_AssembleRawAngle(uint8_t highByte, uint8_t lowByte) {
    /* Mask bits [7:4]; only lower nibble [3:0] contains valid upper angle bits */
    return ((uint16_t)(highByte & 0x0FU) << 8) | (uint16_t)lowByte;
}

/**
 * @brief  Convert 12-bit raw register value to degrees [0.0, 360.0).
 * @param  rawAngle12bit 12-bit raw value (0 to 4095).
 * @retval Angle in degrees.
 */
static inline float AS5600_Math_RawToDegrees(uint16_t rawAngle12bit) {
    /* Multiply by precomputed (360.0 / 4096.0) factor to avoid floating-point division */
    return (float)(rawAngle12bit & 0x0FFFU) * AS5600_RAW_TO_DEGREE;
}

/**
 * @brief  Apply zero-offset correction and wrap negative values into [0.0, 360.0).
 * @param  degreeAngle Raw angle in degrees.
 * @param  startAngle  Reference zero angle in degrees.
 * @retval Offset-corrected angle in [0.0, 360.0).
 */
static inline float AS5600_Math_CorrectAngle(float degreeAngle, float startAngle) {
    float corrected = degreeAngle - startAngle;
    if (corrected < 0.0f) {
        corrected += 360.0f;
    }
    return corrected;
}

/**
 * @brief  Determine the quadrant (1 to 4) for a given angle in [0, 360).
 * @param  angleDeg Angle in degrees.
 * @retval AS5600_Quadrant_e (Quadrant_1 to Quadrant_4).
 */
static inline AS5600_Quadrant_e AS5600_Math_GetQuadrant(float angleDeg) {
    if (angleDeg >= 0.0f && angleDeg < 90.0f) {
        return Quadrant_1;
    }
    if (angleDeg >= 90.0f && angleDeg < 180.0f) {
        return Quadrant_2;
    }
    if (angleDeg >= 180.0f && angleDeg < 270.0f) {
        return Quadrant_3;
    }
    return Quadrant_4;
}

/**
 * @brief  Detect quadrant transitions and update the multi-turn counter and totalAngle.
 *
 * @details Determines which quadrant (1–4) correctedAngle falls in, then checks
 *          for full-revolution crossings:
 *          - Q4 → Q1 transition: increment totalTurns (CW revolution)
 *          - Q1 → Q4 transition: decrement totalTurns (CCW revolution)
 *          Updates totalAngle = (totalTurns * 360) + correctedAngle.
 *
 * @param  params Pointer to AS5600_Params_t with correctedAngle already updated.
 */
void AS5600_Math_CheckQuadrant(AS5600_Params_t *params);

/**
 * @brief  Apply dynamic-alpha EMA filter and compute linear distance and velocity.
 * @param  params            Pointer to AS5600_Params_t.
 * @param  filterInitialized Pointer to filter initialization flag (0 = unseeded, 1 = seeded).
 * @param  screwLead_mm      Lead screw pitch in mm/rev (e.g. 2.0f).
 * @param  dt_sec            Sampling interval in seconds (e.g. 0.01f for 100 Hz).
 */
void AS5600_Math_FilterAndCalcDistance(AS5600_Params_t *params,
                                       uint8_t *filterInitialized,
                                       float screwLead_mm,
                                       float dt_sec);

/**
 * @brief  Process a complete raw 12-bit sample through the encoder pipeline.
 * @param  params            Pointer to AS5600_Params_t.
 * @param  rawAngle          12-bit raw angle (0 to 4095).
 * @param  isBootInitialized Pointer to boot initialization flag (0 = first read, 1 = initialized).
 * @param  filterInitialized Pointer to EMA filter initialization flag (0 = first read, 1 = initialized).
 * @param  screwLead_mm      Lead screw pitch in mm/rev (e.g. 2.0f).
 * @param  dt_sec            Sampling period in seconds (e.g. 0.01f).
 */
void AS5600_Math_ProcessSample(AS5600_Params_t *params,
                               uint16_t rawAngle,
                               uint8_t *isBootInitialized,
                               uint8_t *filterInitialized,
                               float screwLead_mm,
                               float dt_sec);

/**
 * @brief  Snap filtered angle immediately to raw totalAngle when motor is stationary
 *         if residual motion EMA lag exceeds snapThresh_deg (e.g. 0.18 deg = ~2 LSB),
 *         eliminating post-move settling tail and zeroing velocity.
 * @param  params          Pointer to AS5600_Params_t.
 * @param  motorStationary True when motor is IDLE or PAUSED.
 * @param  snapThresh_deg  Angle error threshold in degrees above which to snap.
 * @param  screwLead_mm    Lead screw pitch in mm/rev.
 * @retval true if snap was applied, false otherwise.
 */
bool AS5600_Math_SnapIfStationary(AS5600_Params_t *params,
                                  bool motorStationary,
                                  float snapThresh_deg,
                                  float screwLead_mm);

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_AS5600_MATH_H_ */
