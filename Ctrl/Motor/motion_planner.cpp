#include "motion_planner.h"
#include "math.h"


void MotionPlanner::CurrentTracker::Init()
{
    SetCurrentAcc(context->config->ratedCurrentAcc);
}


void MotionPlanner::CurrentTracker::NewTask(int32_t _realCurrent)
{
    currentIntegral = 0;
    trackCurrent = _realCurrent;
}


void MotionPlanner::CurrentTracker::CalcSoftGoal(int32_t _goalCurrent)
{
    int32_t deltaCurrent = _goalCurrent - trackCurrent;

    if (deltaCurrent == 0)
    {
        trackCurrent = _goalCurrent;
    } else if (deltaCurrent > 0)
    {
        if (trackCurrent >= 0)
        {
            CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(currentAcc);
            if ((int32_t) trackCurrent >= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    } else if (deltaCurrent < 0)
    {
        if (trackCurrent <= 0)
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= (int32_t) _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        } else
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t) trackCurrent <= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    }

    goCurrent = (int32_t) trackCurrent;
}


void MotionPlanner::CurrentTracker::CalcCurrentIntegral(int32_t _current)
{
    currentIntegral += _current;
    trackCurrent += currentIntegral / context->CONTROL_FREQUENCY;
    currentIntegral = currentIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::CurrentTracker::SetCurrentAcc(int32_t _currentAcc)
{
    currentAcc = _currentAcc;
}


void MotionPlanner::VelocityTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);
}


void MotionPlanner::VelocityTracker::SetVelocityAcc(int32_t _velocityAcc)
{
    velocityAcc = _velocityAcc;
}


void MotionPlanner::VelocityTracker::NewTask(int32_t _realVelocity)
{
    velocityIntegral = 0;
    trackVelocity = _realVelocity;
}


void MotionPlanner::VelocityTracker::CalcSoftGoal(int32_t _goalVelocity)
{
    int32_t deltaVelocity = _goalVelocity - trackVelocity;

    if (deltaVelocity == 0)
    {
        trackVelocity = _goalVelocity;
    } else if (deltaVelocity > 0)
    {
        if (trackVelocity >= 0)
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    } else if (deltaVelocity < 0)
    {
        if (trackVelocity <= 0)
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        } else
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    goVelocity = (int32_t) trackVelocity;
}


void MotionPlanner::VelocityTracker::CalcVelocityIntegral(int32_t _velocity)
{
    velocityIntegral += _velocity;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);

    /*
     *  Allow to locking-brake when velocity is lower than (speedLockingBrake).
     *  The best value should be (ratedMoveAcc/1000)
     */
    speedLockingBrake = context->config->ratedVelocityAcc / 1000;
}


void MotionPlanner::PositionTracker::SetVelocityAcc(int32_t value)
{
    velocityUpAcc = value;
    velocityDownAcc = value;
    quickVelocityDownAcc = 0.5f / (float) velocityDownAcc;
}


void MotionPlanner::PositionTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    velocityIntegral = 0;
    trackVelocity = real_speed;
    positionIntegral = 0;
    trackPosition = real_location;
}


void MotionPlanner::PositionTracker::CalcSoftGoal(int32_t _goalPosition)
{
    int32_t deltaPosition = _goalPosition - trackPosition;

    if (deltaPosition == 0)
    {
        if ((trackVelocity >= -speedLockingBrake) && (trackVelocity <= speedLockingBrake))
        {
            velocityIntegral = 0;
            trackVelocity = 0;
            positionIntegral = 0;
        } else if (trackVelocity > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (trackVelocity < 0)
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    } else
    {
        if (trackVelocity == 0)
        {
            if (deltaPosition > 0)
            {
                CalcVelocityIntegral(velocityUpAcc);
            } else
            {
                CalcVelocityIntegral(-velocityUpAcc);
            }
        } else if ((deltaPosition > 0) && (trackVelocity > 0))
        {
            if (trackVelocity <= context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity < context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityUpAcc);
                        if (trackVelocity >= context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = context->config->ratedVelocity;
                        }
                    } else if (trackVelocity > context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(-velocityDownAcc);
                    if (trackVelocity <= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(-velocityDownAcc);
                if (trackVelocity <= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity < 0))
        {
            if (trackVelocity >= -context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t) ((float) trackVelocity *
                                                     (float) trackVelocity *
                                                     (float) quickVelocityDownAcc);
                if (abs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity > -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityUpAcc);
                        if (trackVelocity <= -context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = -context->config->ratedVelocity;
                        }
                    } else if (trackVelocity < -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityDownAcc);
                    }
                } else
                {
                    CalcVelocityIntegral(velocityDownAcc);
                    if (trackVelocity >= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            } else
            {
                CalcVelocityIntegral(velocityDownAcc);
                if (trackVelocity >= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        } else if ((deltaPosition < 0) && (trackVelocity > 0))
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        } else if (((deltaPosition > 0) && (trackVelocity < 0)))
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    CalcPositionIntegral(trackVelocity);

    go_location = (int32_t) trackPosition;
    go_velocity = (int32_t) trackVelocity;
}


void MotionPlanner::PositionTracker::CalcPositionIntegral(int32_t value)
{
    positionIntegral += value;
    trackPosition += positionIntegral / context->CONTROL_FREQUENCY;
    positionIntegral = positionIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionTracker::CalcVelocityIntegral(int32_t value)
{
    velocityIntegral += value;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}


void MotionPlanner::PositionInterpolator::Init()
{

}


void MotionPlanner::PositionInterpolator::NewTask(int32_t _realPosition, int32_t _realVelocity)
{
    recordPosition = _realPosition;
    recordPositionLast = _realPosition;
    estPosition = _realPosition;
    estVelocity = _realVelocity;
}


void MotionPlanner::PositionInterpolator::CalcSoftGoal(int32_t _goalPosition)
{
    recordPositionLast = recordPosition;
    recordPosition = _goalPosition;

    estPositionIntegral += (((recordPosition - recordPositionLast) * context->CONTROL_FREQUENCY)
                            + ((estVelocity << 6) - estVelocity));
    estVelocity = estPositionIntegral >> 6;
    estPositionIntegral -= (estVelocity << 6);

    estPosition = recordPosition;

    goPosition = estPosition;
    goVelocity = estVelocity;
}


void MotionPlanner::TrajectoryTracker::SetSlowDownVelocityAcc(int32_t value)
{
    velocityDownAcc = value;
}


void MotionPlanner::TrajectoryTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    updateTime = 0;
    overtimeFlag = false;
    holdFlag = false;
    dynamicVelocityAccRemainder = 0;
    velocityNow = real_speed;
    velovityNowRemainder = 0;
    positionNow = real_location;
}


void MotionPlanner::TrajectoryTracker::CalcSoftGoal(int32_t _goalPosition, int32_t _goalVelocity)
{
    if (_goalVelocity != recordVelocity || _goalPosition != recordPosition)
    {
        updateTime = 0;
        recordVelocity = _goalVelocity;
        recordPosition = _goalPosition;
        holdFlag = false;   // a new set-point releases the arrival latch

        // Solve the segment's constant accel with divide-by-zero guard + magnitude clamp.
        // Clamp in float domain first to avoid int32 conversion overflow/UB.
        int32_t deltaP = _goalPosition - positionNow;
        int32_t maxAcc = context->config->ratedVelocityAcc;
        if (deltaP == 0)
        {
            // Target is at the current position: cannot infer a boundary-value accel,
            // fall back to a bounded best-effort toward the goal velocity.
            dynamicVelocityAcc = (_goalVelocity == velocityNow) ? 0
                                 : ((_goalVelocity > velocityNow) ? maxAcc : -maxAcc);
        } else
        {
            float a = (float) (_goalVelocity + velocityNow) *
                      (float) (_goalVelocity - velocityNow) /
                      (float) (2 * deltaP);
            if (a > (float) maxAcc) a = (float) maxAcc;
            else if (a < -(float) maxAcc) a = -(float) maxAcc;
            dynamicVelocityAcc = (int32_t) a;
        }

        overtimeFlag = false;
    } else
    {
        if (updateTime >= (updateTimeout * 1000))
            overtimeFlag = true;
        else
            updateTime += context->CONTROL_PERIOD;
    }

    // Latched: hold exactly at goalPosition, skip all integration until a new set-point.
    if (holdFlag)
    {
        goPosition = positionNow;   // == goalPosition
        goVelocity = 0;
        return;
    }

    int32_t vBefore = velocityNow;   // tick-start velocity, for zero-crossing arrival detection

    if (overtimeFlag)
    {
        if (velocityNow == 0)
        {
            dynamicVelocityAccRemainder = 0;
        } else if (velocityNow > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (velocityNow <= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        } else
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (velocityNow >= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        }
    } else
    {
        CalcVelocityIntegral(dynamicVelocityAcc);
    }

    // Zero-velocity goal: latch on ARRIVAL so the motor stops EXACTLY at p instead of
    // integrating past it and relying on the 200ms timeout. Trigger on whichever comes
    // first while approaching p:
    //  - velocity crossing 0 : exact decel; the semi-implicit position integral under-shoots
    //                          p by <= v0/(2*CONTROL_FREQUENCY), so snap forward closes it.
    //  - position crossing p : accel was clamped, arriving at p with residual velocity.
    // In both cases |p - positionNow| is tiny (<= about one tick of motion), so snapping
    // positionNow to goalPosition is safe and jerk-free.
    if (!overtimeFlag && _goalVelocity == 0)
    {
        int32_t errBefore = _goalPosition - positionNow;
        bool approaching = ((int64_t) vBefore * errBefore > 0);        // moving toward target
        bool velCrossed  = (vBefore > 0 && velocityNow <= 0) ||
                           (vBefore < 0 && velocityNow >= 0);          // speed hit 0 this tick
        CalcPositionIntegral(velocityNow);
        int32_t errAfter = _goalPosition - positionNow;
        bool posCrossed  = ((int64_t) errBefore * errAfter <= 0);      // reached/passed p
        if (approaching && (velCrossed || posCrossed))
        {
            positionNow = _goalPosition;
            velocityNow = 0;
            velovityNowRemainder = 0;
            dynamicVelocityAccRemainder = 0;
            holdFlag = true;
        }
    } else
    {
        CalcPositionIntegral(velocityNow);
    }

    goPosition = positionNow;
    goVelocity = velocityNow;
}


void MotionPlanner::TrajectoryTracker::CalcVelocityIntegral(int32_t value)
{
    dynamicVelocityAccRemainder += value; // sum up last remainder
    velocityNow += dynamicVelocityAccRemainder / context->CONTROL_FREQUENCY;
    dynamicVelocityAccRemainder = dynamicVelocityAccRemainder % context->CONTROL_FREQUENCY; // calc remainder
}


void MotionPlanner::TrajectoryTracker::CalcPositionIntegral(int32_t value)
{
    velovityNowRemainder += value;
    positionNow += velovityNowRemainder / context->CONTROL_FREQUENCY;
    velovityNowRemainder = velovityNowRemainder % context->CONTROL_FREQUENCY;
}


void MotionPlanner::TrajectoryTracker::Init(int32_t _updateTimeout)
{
    //SetSlowDownVelocityAcc(context->config->ratedVelocityAcc / 10);
    SetSlowDownVelocityAcc(context->config->ratedVelocityAcc);
    updateTimeout = _updateTimeout;
}


void MotionPlanner::AttachConfig(MotionPlanner::Config_t* _config)
{
    config = _config;

    currentTracker.Init();
    velocityTracker.Init();
    positionTracker.Init();
    positionInterpolator.Init();
    trajectoryTracker.Init(200);
}
