/**
 * @file    as5600_math.c
 * @brief   Implementation of AS5600 angle odometry, multi-turn tracking, and EMA filter.
 * @author  eyazici
 * @date    2026
 */

#include "as5600_math.h"
#include <math.h>
#include <string.h>

void AS5600_Math_CheckQuadrant(AS5600_Params_t *params) {
    if (params == NULL) {
        return;
    }

    /* Determine which 90° quadrant the corrected angle falls in */
    params->prevQuadrant = params->recentQuadrant;
    params->recentQuadrant = AS5600_Math_GetQuadrant(params->correctedAngle);

    /*
     * Detect full-revolution boundary crossings by checking for
     * transitions between Q4 and Q1 (the 360°/0° boundary).
     */
    if (params->prevQuadrant == Quadrant_4 && params->recentQuadrant == Quadrant_1) {
        /* CW full revolution detected: Q4 → Q1 crossing (past 360°/0° boundary) */
        params->prevTurns = params->totalTurns;
        params->totalTurns++;
    }
    if (params->prevQuadrant == Quadrant_1 && params->recentQuadrant == Quadrant_4) {
        /* CCW full revolution detected: Q1 → Q4 crossing (past 0°/360° boundary) */
        params->prevTurns = params->totalTurns;
        params->totalTurns--;
    }

    /* Compute cumulative absolute angle across all revolutions */
    params->totalAngle = ((float)params->totalTurns * 360.0f) + params->correctedAngle;
}

void AS5600_Math_FilterAndCalcDistance(AS5600_Params_t *params,
                                       uint8_t *filterInitialized,
                                       float screwLead_mm,
                                       float dt_sec) {
    if (params == NULL || filterInitialized == NULL) {
        return;
    }

    if (!(*filterInitialized)) {
        params->filteredTotalAngle = params->totalAngle;
        *filterInitialized = 1;
    } else {
        /* EMA with dynamic alpha */
        float error = fabsf(params->totalAngle - params->filteredTotalAngle);

        /* Snap to raw angle when stationary (error < AS5600_DYNAMIC_EMA_THRESH_DEG) to eliminate EMA settling lag */
        float dynamic_alpha;
        if (error < AS5600_DYNAMIC_EMA_THRESH_DEG) {
            dynamic_alpha = 1.0f;
        } else {
            dynamic_alpha = AS5600_EMA_ALPHA;
        }

        params->filteredTotalAngle = (dynamic_alpha * params->totalAngle) +
                                     ((1.0f - dynamic_alpha) * params->filteredTotalAngle);
    }

    float last_distance = params->travelledDistance_mm;
    params->travelledDistance_mm = (params->filteredTotalAngle / 360.0f) * screwLead_mm;

    if (dt_sec > 0.0f) {
        float raw_velocity_mm_s = (params->travelledDistance_mm - last_distance) / dt_sec;
        /* Responsive velocity EMA filter (alpha = 0.33) suppresses 10 ms differentiation noise
         * while reducing decay latency to ~25 ms for rapid stall detection */
        params->currentVelocity_mm_s = (AS5600_VEL_EMA_ALPHA * raw_velocity_mm_s) +
                                       ((1.0f - AS5600_VEL_EMA_ALPHA) * params->currentVelocity_mm_s);
    }
}

void AS5600_Math_ProcessSample(AS5600_Params_t *params,
                               uint16_t rawAngle,
                               uint8_t *isBootInitialized,
                               uint8_t *filterInitialized,
                               float screwLead_mm,
                               float dt_sec) {
    if (params == NULL || isBootInitialized == NULL || filterInitialized == NULL) {
        return;
    }

    params->rawAngle = rawAngle & 0x0FFFU;
    params->degreeAngle = (float)params->rawAngle * AS5600_RAW_TO_DEGREE;

    if (!(*isBootInitialized)) {
        params->startAngle = params->degreeAngle;
        *isBootInitialized = 1;
    }

    params->correctedAngle = AS5600_Math_CorrectAngle(params->degreeAngle, params->startAngle);
    AS5600_Math_CheckQuadrant(params);
    AS5600_Math_FilterAndCalcDistance(params, filterInitialized, screwLead_mm, dt_sec);
}

bool AS5600_Math_SnapIfStationary(AS5600_Params_t *params,
                                  bool motorStationary,
                                  float snapThresh_deg,
                                  float screwLead_mm) {
    if (params == NULL || !motorStationary) {
        return false;
    }
    if (fabsf(params->totalAngle - params->filteredTotalAngle) > snapThresh_deg) {
        params->filteredTotalAngle = params->totalAngle;
        params->travelledDistance_mm = (params->filteredTotalAngle / 360.0f) * screwLead_mm;
        params->currentVelocity_mm_s = 0.0f;
        return true;
    }
    return false;
}
