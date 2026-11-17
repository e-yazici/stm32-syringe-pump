/**
 * @file    pump_kinematics.h
 * @brief   Hardware-agnostic volumetric kinematics, unit conversions, and pump calculations.
 * @author  eyazici
 * @date    2026
 *
 * @details Provides pure mathematical functions for converting user-facing
 *          pump parameters (volume, flow rate, syringe dimensions) into physical
 *          motion parameters (linear travel, speed, steps, frequency, acceleration).
 *
 *          This module has ZERO hardware or RTOS dependencies and is 100% unit-testable.
 */

#ifndef LIBS_INC_PUMP_KINEMATICS_H_
#define LIBS_INC_PUMP_KINEMATICS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MATH_PI
#define MATH_PI 3.14159265358979323846f
#endif

#define VOL_ML_TO_MM3       1000.0f   ///< Conversion factor: 1 mL = 1000 mm³
#define TIME_MIN_TO_SEC     60.0f     ///< Conversion factor: 1 min = 60 sec

/** @brief Volume unit selection */
typedef enum {
    PUMP_VOL_UNIT_UL = 0,
    PUMP_VOL_UNIT_ML = 1
} PumpVolumeUnit_e;

/** @brief Flow rate unit selection */
typedef enum {
    PUMP_FLOW_UNIT_UL_MIN = 0,
    PUMP_FLOW_UNIT_ML_MIN = 1,
    PUMP_FLOW_UNIT_UL_HR  = 2,
    PUMP_FLOW_UNIT_ML_HR  = 3
} PumpFlowUnit_e;

/** @brief Physical hardware specs of the syringe pump lead screw and motor */
typedef struct {
    float screwLead_mm_per_rev;    ///< Lead screw pitch in mm/rev (e.g. 2.0 mm)
    uint16_t motorStepsPerRev;     ///< Full steps per revolution (e.g. 200 for 1.8°)
    uint16_t microstepping;        ///< Microstepping divisor (e.g. 128)
} PumpHwConfig_t;

/** @brief Calculated motor kinematics output */
typedef struct {
    float area_mm2;                ///< Syringe cross-sectional area (mm²)
    float linearTravel_mm;         ///< Total linear travel distance (mm)
    float linearSpeed_mm_sec;      ///< Linear plunger travel speed (mm/s)
    float stepsPerMm;              ///< Microsteps required per mm of linear travel
    uint32_t totalSteps;           ///< Total microsteps required for the job
    float cruiseFreqHz;            ///< Pulse frequency at cruising speed (Hz)
    uint32_t accelSteps;           ///< Steps allocated for acceleration ramp
} PumpKinematics_t;

/**
 * @brief  Convert user volume value to cubic millimeters (mm³).
 * @param  volume Value entered by user.
 * @param  unit   Unit of the value (mL or µL). Note: 1 µL = 1 mm³.
 * @retval Volume in mm³.
 */
static inline float pump_vol_to_mm3(float volume, PumpVolumeUnit_e unit) {
    if (unit == PUMP_VOL_UNIT_ML) {
        return volume * VOL_ML_TO_MM3;
    }
    /* 1 uL is physically equivalent to 1 mm3; 1 mL = 1000 mm3 */
    return volume;
}

/**
 * @brief  Convert user flow rate value to mm³/sec.
 * @param  flowRate Value entered by user.
 * @param  unit     Unit of the flow rate (mL/min, µL/min, mL/hr, µL/hr).
 * @retval Flow rate in mm³/sec.
 */
float pump_flow_to_mm3_sec(float flowRate, PumpFlowUnit_e unit);

/**
 * @brief  Convert user flow rate value to mL/min (useful for UI displays and estimates).
 * @param  flowRate Value entered by user.
 * @param  unit     Unit of the flow rate.
 * @retval Flow rate in mL/min.
 */
float pump_flow_to_ml_min(float flowRate, PumpFlowUnit_e unit);

/**
 * @brief  Calculate syringe barrel internal cross-sectional area (A = π * (d/2)²).
 * @param  diameter_mm Inner diameter in mm.
 * @retval Area in mm², or 0.0f if diameter <= 0.
 */
static inline float pump_calc_cross_section_area(float diameter_mm) {
    if (diameter_mm <= 0.0f) {
        return 0.0f;
    }
    float r = diameter_mm / 2.0f;
    return MATH_PI * r * r;
}

/**
 * @brief  Calculate linear travel distance required for a given volume.
 * @param  vol_mm3  Volume in mm³.
 * @param  area_mm2 Barrel area in mm².
 * @retval Linear travel in mm.
 */
static inline float pump_calc_linear_travel(float vol_mm3, float area_mm2) {
    if (area_mm2 <= 0.0f) {
        return 0.0f;
    }
    return vol_mm3 / area_mm2;
}

/**
 * @brief  Calculate linear speed for a given flow rate.
 * @param  flow_mm3_sec Flow rate in mm³/sec.
 * @param  area_mm2     Barrel area in mm².
 * @retval Linear speed in mm/sec.
 */
static inline float pump_calc_linear_speed(float flow_mm3_sec, float area_mm2) {
    if (area_mm2 <= 0.0f) {
        return 0.0f;
    }
    return flow_mm3_sec / area_mm2;
}

/**
 * @brief  Calculate microsteps per mm of linear motion.
 * @param  hw Pointer to hardware specs.
 * @retval Steps per mm.
 */
static inline float pump_calc_steps_per_mm(const PumpHwConfig_t *hw) {
    if (hw == NULL || hw->screwLead_mm_per_rev <= 0.0f) {
        return 0.0f;
    }
    float baseSteps = (float)hw->motorStepsPerRev / hw->screwLead_mm_per_rev;
    return baseSteps * (float)hw->microstepping;
}

/**
 * @brief  Calculate acceleration ramp step count using kinematic formula S = V² / (2 * A).
 * @param  cruiseHz        Target cruise step frequency in Hz.
 * @param  targetAccelRate Acceleration rate in steps/sec².
 * @param  minSteps        Minimum acceleration steps constraint.
 * @param  maxSteps        Maximum acceleration steps constraint.
 * @param  totalSteps      Total steps of the move (to detect triangular profile).
 * @retval Number of acceleration steps to use.
 */
uint32_t pump_calc_accel_steps(float cruiseHz, float targetAccelRate,
                               uint32_t minSteps, uint32_t maxSteps,
                               uint32_t totalSteps);

/**
 * @brief  Compute the full kinematics pipeline for a pump job.
 * @param  volume          Target volume.
 * @param  volUnit         Volume unit (mL or µL).
 * @param  flowRate        Target flow rate.
 * @param  flowUnit        Flow rate unit.
 * @param  diameter_mm     Syringe barrel inner diameter in mm.
 * @param  hw              Hardware configuration.
 * @param  targetAccelRate Acceleration rate in steps/sec².
 * @param  minAccelSteps   Minimum acceleration steps.
 * @param  maxAccelSteps   Maximum acceleration steps.
 * @param  out             Pointer to output structure to populate.
 * @retval true on success, false if invalid parameters (e.g. zero diameter, zero flow).
 */
bool pump_calc_kinematics(float volume, PumpVolumeUnit_e volUnit,
                          float flowRate, PumpFlowUnit_e flowUnit,
                          float diameter_mm, const PumpHwConfig_t *hw,
                          float targetAccelRate, uint32_t minAccelSteps, uint32_t maxAccelSteps,
                          PumpKinematics_t *out);

/**
 * @brief  Reverse-calculate live pumped volume in mL from completed step count.
 * @param  stepsDone   Number of steps executed so far.
 * @param  stepsPerMm  Steps per mm ratio.
 * @param  area_mm2    Syringe barrel cross-sectional area in mm².
 * @retval Volume pumped so far in mL.
 */
static inline float pump_calc_pumped_volume_ml(uint32_t stepsDone, float stepsPerMm, float area_mm2) {
    if (stepsPerMm <= 0.0f) {
        return 0.0f;
    }
    float pumped_mm = (float)stepsDone / stepsPerMm;
    return (pumped_mm * area_mm2) / VOL_ML_TO_MM3;
}

/**
 * @brief  Calculate estimated remaining time in seconds.
 * @param  remainingVol_ml Remaining volume in mL.
 * @param  flowRate_ml_min Target flow rate in mL/min.
 * @retval Remaining time in seconds.
 */
static inline float pump_calc_remaining_time_sec(float remainingVol_ml, float flowRate_ml_min) {
    if (flowRate_ml_min <= 0.0f || remainingVol_ml <= 0.0f) {
        return 0.0f;
    }
    return (remainingVol_ml / flowRate_ml_min) * TIME_MIN_TO_SEC;
}

/**
 * @brief  Calculate syringe inner diameter from the weight of water delivered during calibration
 *         using ISO 8655 gravimetric formula V = m * Z.
 * @param  weight_g    Measured mass (m) of delivered water in grams.
 * @param  z_factor    ISO 8655 conversion factor (Z) in uL/mg (or mL/g, e.g. ~1.0029).
 * @param  distance_mm Calibration travel distance in mm.
 * @retval Calculated inner diameter in mm, or 0.0f if any input <= 0.
 */
float pump_calc_diameter_from_weight(float weight_g, float z_factor, float distance_mm);

/**
 * @brief  Calculate direction-aware relative linear travel from encoder baseline (clamped >= 0.0f).
 * @param  currentDist_mm  Current encoder travelledDistance_mm.
 * @param  baselineDist_mm Baseline encoder distance captured at job start.
 * @param  isForward       True for forward direction, false for backward.
 * @retval Relative travel distance in mm (>= 0.0f).
 */
float pump_calc_relative_travel_mm(float currentDist_mm, float baselineDist_mm, bool isForward);

/**
 * @brief  Calculate remaining microsteps from target distance and actual encoder travel on resume.
 * @param  targetDist_mm   Total target linear travel in mm.
 * @param  actualTravel_mm Ground-truth relative travel completed so far in mm.
 * @param  stepsPerMm      Microsteps per mm conversion factor.
 * @retval Remaining microsteps (clamped >= 0).
 */
uint32_t pump_calc_remaining_steps(float targetDist_mm, float actualTravel_mm, float stepsPerMm);

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_PUMP_KINEMATICS_H_ */
