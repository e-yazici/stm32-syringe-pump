/**
 * @file    pump_kinematics.c
 * @brief   Implementation of hardware-agnostic volumetric kinematics and pump calculations.
 * @author  eyazici
 * @date    2026
 */

#include "pump_kinematics.h"
#include <stddef.h>
#include <math.h>



float pump_flow_to_mm3_sec(float flowRate, PumpFlowUnit_e unit) {
    float flow_mm3_min = 0.0f;
    switch (unit) {
        case PUMP_FLOW_UNIT_ML_MIN:
            flow_mm3_min = flowRate * VOL_ML_TO_MM3;
            break;
        case PUMP_FLOW_UNIT_UL_MIN:
            flow_mm3_min = flowRate;
            break;
        case PUMP_FLOW_UNIT_ML_HR:
            flow_mm3_min = (flowRate * VOL_ML_TO_MM3) / TIME_MIN_TO_SEC;
            break;
        case PUMP_FLOW_UNIT_UL_HR:
            flow_mm3_min = flowRate / TIME_MIN_TO_SEC;
            break;
        default:
            flow_mm3_min = 0.0f;
            break;
    }
    return flow_mm3_min / TIME_MIN_TO_SEC;
}

float pump_flow_to_ml_min(float flowRate, PumpFlowUnit_e unit) {
    switch (unit) {
        case PUMP_FLOW_UNIT_ML_MIN:
            return flowRate;
        case PUMP_FLOW_UNIT_UL_MIN:
            return flowRate / VOL_ML_TO_MM3;
        case PUMP_FLOW_UNIT_ML_HR:
            return flowRate / TIME_MIN_TO_SEC;
        case PUMP_FLOW_UNIT_UL_HR:
            return flowRate / (VOL_ML_TO_MM3 * TIME_MIN_TO_SEC);
        default:
            return 0.0f;
    }
}


uint32_t pump_calc_accel_steps(float cruiseHz, float targetAccelRate,
                               uint32_t minSteps, uint32_t maxSteps,
                               uint32_t totalSteps) {
    if (targetAccelRate <= 0.0f) {
        return minSteps;
    }

    /* S = V² / (2 * A) */
    uint32_t accel = (uint32_t)((cruiseHz * cruiseHz) / (2.0f * targetAccelRate));
    if (accel > maxSteps) {
        accel = maxSteps;
    }
    if (accel < minSteps) {
        accel = minSteps;
    }

    /* If total move is too short for full trapezoid, clamp to triangular profile */
    if (totalSteps < (2U * accel)) {
        accel = totalSteps / 2U;
    }

    return accel;
}

bool pump_calc_kinematics(float volume, PumpVolumeUnit_e volUnit,
                          float flowRate, PumpFlowUnit_e flowUnit,
                          float diameter_mm, const PumpHwConfig_t *hw,
                          float targetAccelRate, uint32_t minAccelSteps, uint32_t maxAccelSteps,
                          PumpKinematics_t *out) {
    if (out == NULL || hw == NULL || diameter_mm <= 0.0f || volume <= 0.0f || flowRate <= 0.0f) {
        return false;
    }

    float area = pump_calc_cross_section_area(diameter_mm);
    if (area <= 0.0f) {
        return false;
    }

    float vol_mm3 = pump_vol_to_mm3(volume, volUnit);
    float flow_mm3_sec = pump_flow_to_mm3_sec(flowRate, flowUnit);

    float linearTravel_mm = pump_calc_linear_travel(vol_mm3, area);
    float linearSpeed_mm_sec = pump_calc_linear_speed(flow_mm3_sec, area);
    float stepsPerMm = pump_calc_steps_per_mm(hw);

    uint32_t totalSteps = (uint32_t)(linearTravel_mm * stepsPerMm);
    float cruiseHz = linearSpeed_mm_sec * stepsPerMm;

    uint32_t accelSteps = pump_calc_accel_steps(cruiseHz, targetAccelRate,
                                                minAccelSteps, maxAccelSteps,
                                                totalSteps);

    out->area_mm2 = area;
    out->linearTravel_mm = linearTravel_mm;
    out->linearSpeed_mm_sec = linearSpeed_mm_sec;
    out->stepsPerMm = stepsPerMm;
    out->totalSteps = totalSteps;
    out->cruiseFreqHz = cruiseHz;
    out->accelSteps = accelSteps;

    return true;
}



float pump_calc_diameter_from_weight(float weight_g, float z_factor, float distance_mm) {
    if (weight_g <= 0.0f || z_factor <= 0.0f || distance_mm <= 0.0f) {
        return 0.0f;
    }
    /* ISO 8655 gravimetric calibration: V (mL) = m (g) * Z (mL/g or uL/mg) */
    /* 1 mL = 1000 mm3 (VOL_ML_TO_MM3) */
    float volume_mm3 = weight_g * z_factor * VOL_ML_TO_MM3;
    /* Area = Volume / Distance */
    float area = volume_mm3 / distance_mm;
    /* Diameter = 2 * sqrt(Area / π) */
    return 2.0f * sqrtf(area / MATH_PI);
}

float pump_calc_relative_travel_mm(float currentDist_mm, float baselineDist_mm, bool isForward) {
    float rel = isForward ? (currentDist_mm - baselineDist_mm) : (baselineDist_mm - currentDist_mm);
    return (rel > 0.0f) ? rel : 0.0f;
}

uint32_t pump_calc_remaining_steps(float targetDist_mm, float actualTravel_mm, float stepsPerMm) {
    if (stepsPerMm <= 0.0f) {
        return 0U;
    }
    float clampedActual = (actualTravel_mm > 0.0f) ? actualTravel_mm : 0.0f;
    float remainingDist_mm = targetDist_mm - clampedActual;
    if (remainingDist_mm <= 0.0f) {
        return 0U;
    }
    return (uint32_t)lroundf(remainingDist_mm * stepsPerMm);
}
