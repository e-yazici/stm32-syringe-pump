/**
 * @file    closed_loop.h
 * @brief   Closed-loop motion supervisor, step slip compensation, and stall detection.
 * @author  eyazici
 * @date    2026
 *
 * @details Platform-independent closed-loop domain module that monitors linear
 *          travel distance and velocity from AS5600 encoder feedback against
 *          commanded motion profile parameters.
 *
 *          Detects:
 *          - Deadband: E_lag <= positionLagThresh_mm (default 0.025 mm = 320 microsteps).
 *          - Step skip (telemetry only): positionLagThresh_mm < E_lag, with velocity
 *            maintained (V_act >= stallVelRatioThresh * V_cruise). ClosedLoop_Update()
 *            computes compensationSteps for diagnostic/telemetry (`gClosedLoopDebug`),
 *            while mid-cruise step injection is not applied (under-travel is corrected
 *            by post-move creep).
 *          - Motor stall: E_lag >= stallLagThresh_mm (default 0.030 mm = ~3 full steps)
 *            AND V_act < stallVelRatioThresh * V_cruise (default 0.15 = 15%)
 *            persistently for >= effective stallPersistTime_ms (40 ms baseline,
 *            20 ms at >= 0.50 mm/s, up to 1000 ms below 0.05 mm/s).
 *          - Blanking window: Disables stall detection for blankingDuration_ms
 *            (default 250 ms) upon start or resume.
 *
 *          Zero hardware or RTOS dependencies: fully host unit-testable.
 */

#ifndef LIBS_INC_CLOSED_LOOP_H_
#define LIBS_INC_CLOSED_LOOP_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default Configuration Constants (overridden by CMake: 1 in Debug, 0 in Release) */
#ifndef CLOSED_LOOP_TEST
#ifdef DEBUG
#define CLOSED_LOOP_TEST                1       ///< Enable Cortex-Debug Live Watch telemetry struct in Debug builds
#else
#define CLOSED_LOOP_TEST                0       ///< Disable Live Watch telemetry struct in Release builds
#endif
#endif

#define CL_DEFAULT_DEADBAND_MM          0.025f  ///< Deadband threshold (0.025 mm = 320 microsteps @ 12800 steps/mm, ~2.5 full steps)
#define CL_DEFAULT_STALL_LAG_MM         0.030f  ///< Blockage stall lag threshold (0.030 mm = ~3 full steps)
#define CL_DEFAULT_GROSS_STALL_LAG_MM   0.0f    ///< Disabled (0.0f) for chem lab: allows unlimited slip compensation as long as motor turns
#define CL_DEFAULT_STALL_VEL_RATIO      0.15f   ///< Velocity ratio threshold (15% of cruise velocity)
#define CL_DEFAULT_STALL_PERSIST_MS     40U     ///< Baseline stall persistence for moderate speeds (40 ms; adapts to 20 ms at high speed)
#define CL_DEFAULT_BLANKING_MS          250U    ///< Initial acceleration blanking window (250 ms)
#define CL_DEFAULT_MAX_COMP_STEPS       128000U ///< Generous ceiling for chem lab compensation (128,000 steps = 10.0 mm @ 12800 steps/mm)
#define CL_DEFAULT_CREEP_LAG_MM         0.02f   ///< Creep triggering deadband threshold (0.02 mm = 256 microsteps)
#define CL_DEFAULT_CREEP_MAX_STEPS      128000U ///< Maximum allowable creep compensation steps (128,000 microsteps = 10.0 mm)
#define CL_CREEP_FREQ_HZ                3000.0f ///< Low-speed creep step frequency in Hz (matches START_FREQ_HZ for max torque)

/**
 * @brief Configuration parameters for closed-loop supervisor.
 */
typedef struct {
    float    positionLagThresh_mm;  ///< Deadband threshold for minor slip (default 0.025 mm)
    float    stallLagThresh_mm;     ///< Position lag threshold for low-velocity stall (default 0.030 mm)
    float    grossStallLagThresh_mm;///< Gross position lag threshold for hard stall (default 0.0f = disabled)
    float    stallVelRatioThresh;   ///< Velocity ratio threshold (default 0.15 = 15% cruise)
    uint32_t stallPersistTime_ms;   ///< Baseline persistence duration before stall trip (default 40 ms)
    uint32_t blankingDuration_ms;   ///< Blanking window upon start/resume (default 250 ms)
    uint32_t maxCompensationSteps;  ///< Optional safety ceiling for compensation steps (0 = default clamp)
} ClosedLoopConfig_t;

/**
 * @brief Status enumeration returned by closed-loop update.
 */
typedef enum {
    CL_STATUS_NORMAL,               ///< Motion within nominal bounds
    CL_STATUS_STEP_SKIP_DETECTED,   ///< Step skip detected (diagnostic/telemetry only; physical lag corrected by post-move creep)
    CL_STATUS_STALL_DETECTED        ///< Severe lag/blockage confirmed, immediate halt required
} ClosedLoopStatus_e;

/**
 * @brief Output result structure from closed-loop update.
 */
typedef struct {
    ClosedLoopStatus_e status;             ///< Overall status evaluation
    uint32_t           compensationSteps;  ///< Diagnostic/telemetry step deficit (not injected during cruise; corrected by post-move creep)
    float              positionLag_mm;     ///< Commanded - Actual distance (mm)
    float              velocityRatio;      ///< Actual / Commanded velocity
    bool               blankingActive;     ///< True during initial acceleration window
} ClosedLoopOutput_t;

/**
 * @brief  Initialize closed-loop controller with configuration parameters.
 * @param  cfg Pointer to configuration structure, or NULL to use defaults.
 */
void ClosedLoop_Init(const ClosedLoopConfig_t *cfg);

/**
 * @brief  Reset all internal state variables (stall timers, blanking, latch).
 */
void ClosedLoop_Reset(void);

/**
 * @brief  Arm the acceleration blanking window.
 * @param  duration_ms Blanking duration in ms (0 disables blanking).
 */
void ClosedLoop_ArmBlanking(uint32_t duration_ms);

/**
 * @brief  Execute periodic closed-loop supervisory update cycle.
 * @param  commandedDist_mm Commanded linear position in mm.
 * @param  actualDist_mm    Actual encoder position in mm.
 * @param  commandedVel_mm_s Commanded cruise velocity in mm/s.
 * @param  actualVel_mm_s   Actual encoder velocity in mm/s.
 * @param  dt_ms            Elapsed time in milliseconds since last update.
 * @param  stepsPerMm       Microsteps per millimeter conversion factor.
 * @retval ClosedLoopOutput_t Evaluated status, lag, velocity ratio, and compensation steps.
 */
ClosedLoopOutput_t ClosedLoop_Update(float commandedDist_mm, float actualDist_mm,
                                    float commandedVel_mm_s, float actualVel_mm_s,
                                    uint32_t dt_ms, uint32_t stepsPerMm);

/**
 * @brief  Calculate post-move creep correction microsteps from target and actual distance.
 *
 * @param  targetDist_mm Commanded final travel distance in mm.
 * @param  actualDist_mm Measured final encoder distance in mm.
 * @param  lagThresh_mm  Deadband threshold for creep triggering (e.g. 0.02 mm).
 * @param  maxCreepSteps Maximum allowable creep steps ceiling (e.g. 128000).
 * @param  stepsPerMm    Microsteps per millimeter conversion factor.
 * @retval Number of compensation microsteps to execute (forward only), or 0 if within threshold.
 */
uint32_t ClosedLoop_CalcCreepSteps(float targetDist_mm, float actualDist_mm,
                                   float lagThresh_mm, uint32_t maxCreepSteps,
                                   uint32_t stepsPerMm);

/**
 * @brief  Set maximum allowable compensation steps ceiling.
 * @param  maxSteps Upper bound on compensation steps (0 = use default).
 */
void ClosedLoop_SetMaxCompensationSteps(uint32_t maxSteps);

/**
 * @brief  Get current active configuration.
 * @retval Pointer to internal configuration structure.
 */
const ClosedLoopConfig_t* ClosedLoop_GetConfig(void);

/**
 * @brief  Get current accumulated stall persistence timer.
 * @retval Elapsed stall persistence time in ms.
 */
uint32_t ClosedLoop_GetStallTimer(void);

/**
 * @brief  Get remaining blanking duration.
 * @retval Remaining blanking time in ms.
 */
uint32_t ClosedLoop_GetBlankingTimer(void);

/**
 * @brief  Check if stall state is currently latched.
 * @retval true if stalled, false otherwise.
 */
bool ClosedLoop_IsStalled(void);

#if CLOSED_LOOP_TEST
/**
 * @brief Real-time hardware telemetry structure for Cortex-Debug (J-Link) Live Watch.
 *        Watch `gClosedLoopDebug` in VS Code Cortex Live Watch.
 */
typedef struct {
    /* --- Supervisory Inputs & Real AS5600 Odometry --- */
    float              targetDist_mm;          ///< Total planned linear travel (mm)
    float              baselineDist_mm;        ///< Encoder zero reference at start/resume (mm)
    float              commandedDist_mm;       ///< Expected linear travel from stepsDone (mm)
    float              actualDist_mm;          ///< Measured AS5600 travel from baseline (mm)
    float              commandedVel_mm_s;      ///< Target cruise speed (mm/s)
    float              actualVel_mm_s;         ///< Measured AS5600 velocity (mm/s)

    /* --- Closed-Loop Outputs & Timers --- */
    float              positionLag_mm;         ///< E_lag = commandedDist_mm - actualDist_mm (mm)
    float              velocityRatio;          ///< actualVel_mm_s / commandedVel_mm_s
    ClosedLoopStatus_e status;                 ///< 0=NORMAL, 1=STEP_SKIP_DETECTED, 2=STALL_DETECTED
    uint32_t           stallTimer_ms;          ///< Accumulated unblanked stall time (40 ms baseline, speed-adaptive 20/40/1000 ms)
    uint32_t           blankingTimer_ms;       ///< Remaining startup/resume blanking time (starts at 250 ms)
    bool               blankingActive;         ///< True while blanking window is counting down
    bool               isStalled;              ///< Latched stall flag

    /* --- Stepper & Compensation Counters --- */
    uint32_t           compensationSteps;      ///< Diagnostic step deficit from ClosedLoop_Update (telemetry only; not injected during cruise)
    uint32_t           creepStepsLast;         ///< Microsteps executed by post-move creep (Feature 7)
    uint32_t           stepsDone;              ///< Microsteps executed so far
    uint32_t           stepsRemaining;         ///< Microsteps remaining in gStepperControl
    uint8_t            motorState;             ///< Current gStepperControl.MotorState (0=IDLE,1=ACCEL,2=CRUISE,3=DECEL,4=PAUSED)
    bool               encoderValid;           ///< AS5600 snapshot validity flag
    uint32_t           updateCount;            ///< Increments at 50 Hz while supervisory loop is active
} ClosedLoopDebug_t;

extern volatile ClosedLoopDebug_t gClosedLoopDebug;
#endif /* CLOSED_LOOP_TEST */

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_CLOSED_LOOP_H_ */
