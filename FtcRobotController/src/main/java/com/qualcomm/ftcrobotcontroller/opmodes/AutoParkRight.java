package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to autonomously park the robot.
 */
public class AutoParkRight extends TreadedBotHardware {
    enum RobotState
    {
        Wait,
        MoveForwardShort,
        TurnRight,
        MoveForwardLong,
        Stop
    }

    final double FORWARD_SPEED = 0.30;
    final double TURN_SPEED = -0.15;
    final double MOVEFORWARDSHORT_DURATION = 3.8;
    final double TURNLEFT_DURATION    = 1.7;
    final double MOVEFORWARDLONG_DURATION = 3.7;

    final double SQUELCH_DURATION = 0.5;

    OpticalDistanceSensor opticalDistanceSensor;

    RobotState state;
    StopWatch stopWatch;
    double moveForwardLongDuration;
    double moveForwardShortDuration;
    double turnLeftDuration;
    StopWatch squelch;

    @Override
    public void start() {

        // initialze the state and timestamp
        moveForwardShortDuration = MOVEFORWARDSHORT_DURATION;
        turnLeftDuration         = TURNLEFT_DURATION;
        moveForwardLongDuration  = MOVEFORWARDLONG_DURATION;

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("distance_sensor");
        //opticalDistanceSensor.enableLed(true);

        //Set gripper to open
        openGripper();

        state = RobotState.Wait;
        stopWatch = new StopWatch();
        squelch = new StopWatch();
    }

    @Override
    public void init_loop() {
        // adjust the move forward duration using the up/down dpad
        if ( gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardShortDuration = Math.max(moveForwardShortDuration - 0.01, 0.1);
            squelch.reset();
        }
        if ( gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardShortDuration = Math.min(moveForwardShortDuration + 0.01, 2 * MOVEFORWARDSHORT_DURATION);
            squelch.reset();
        }

        // adjust the turn left duration using the left/right dpad
        if ( gamepad1.dpad_left && squelch.elapsedTime() > SQUELCH_DURATION ) {
            turnLeftDuration = Math.max(turnLeftDuration - 0.01, 0.1);
            squelch.reset();
        }
        if ( gamepad1.dpad_right && squelch.elapsedTime() > SQUELCH_DURATION) {
            turnLeftDuration = Math.min(turnLeftDuration + 0.01, 2 * TURNLEFT_DURATION);
            squelch.reset();
        }

        // adjust the move forward duration1 using the up/down dpad
        if ( gamepad1.a && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardLongDuration = Math.max(moveForwardLongDuration - 0.01, 0.1);
            squelch.reset();
        }
        if ( gamepad1.b && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardLongDuration = Math.min(moveForwardLongDuration + 0.01, 2 * MOVEFORWARDLONG_DURATION);
            squelch.reset();
        }

        telemetry.addData("Durations", String.format("%.1f %.1f %.1f",moveForwardShortDuration, turnLeftDuration, moveForwardLongDuration));
    }

    @Override
    public void loop() {

        if ( gamepad1.a ) {
            state = RobotState.Stop;
            stopWatch.reset();
        }
        if ( state == RobotState.Stop && gamepad1.x) {
            state = RobotState.MoveForwardShort;
            stopWatch.reset();
        }

        switch ( state )
        {
            case Wait:
            {
                if (stopWatch.elapsedTime() > 10.0 ) {
                    state = RobotState.MoveForwardShort;
                    stopWatch.reset();
                }
            }
            break;

            case MoveForwardShort:
            {
                // set  motors to move forward
                setDrivePower(FORWARD_SPEED);

                // check condition to switch
                if ( stopWatch.elapsedTime() > moveForwardShortDuration)
                {
                    state = RobotState.TurnRight;
                    stopWatch.reset();
                }
            }
            break;

            case TurnRight:
            {
                // set motors to turn right
                setDrivePower(TURN_SPEED, -TURN_SPEED);

                //check condition to switch
                if ( stopWatch.elapsedTime() > turnLeftDuration)
                {
                    state = RobotState.MoveForwardLong;
                    stopWatch.reset();
                }
            }
            break;

            case MoveForwardLong:
            {
                // set  motors to move forward
                setDrivePower(FORWARD_SPEED);

                // check condition to switch
                if ( stopWatch.elapsedTime() > moveForwardLongDuration  || opticalDistanceSensor.getLightDetected() > 0.8)
                {
                    state = RobotState.Stop ;
                    stopWatch.reset();
                }
            }
            break;

            case Stop:
            {
                setDrivePower(0);

                if (gamepad1.right_bumper && squelch.elapsedTime() > SQUELCH_DURATION) {
                    moveForwardShortDuration += 0.1;
                    moveForwardShortDuration = Range.clip(moveForwardShortDuration, 0.1, 2 * MOVEFORWARDSHORT_DURATION);
                    squelch.reset();
                }
                if (gamepad1.left_bumper && squelch.elapsedTime() > SQUELCH_DURATION) {
                    moveForwardShortDuration -= 0.1;
                    moveForwardShortDuration = Range.clip(moveForwardShortDuration, 0.1, 2 * MOVEFORWARDSHORT_DURATION);
                    squelch.reset();
                }
                if (gamepad1.dpad_right && squelch.elapsedTime() > SQUELCH_DURATION) {
                    turnLeftDuration += 0.1;
                    turnLeftDuration = Range.clip(turnLeftDuration, 0.1, 2 * TURNLEFT_DURATION);
                    squelch.reset();
                }
                if (gamepad1.dpad_left && squelch.elapsedTime() > SQUELCH_DURATION) {
                    turnLeftDuration -= 0.1;
                    turnLeftDuration = Range.clip(turnLeftDuration, 0.1, 2 * TURNLEFT_DURATION);
                    squelch.reset();
                }
                if (gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
                    moveForwardLongDuration += 0.1;
                    moveForwardLongDuration = Range.clip(moveForwardLongDuration, 0.1, 2 * MOVEFORWARDLONG_DURATION);
                    squelch.reset();
                }
                if (gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
                    moveForwardLongDuration -= 0.1;
                    moveForwardLongDuration = Range.clip(moveForwardLongDuration, 0.1, 2 * MOVEFORWARDLONG_DURATION);
                    squelch.reset();
                }
            }
            break;
        }

        telemetry.addData("State", String.format("%s  elapsed: %.3f", state, stopWatch.elapsedTime()));
        telemetry.addData("Durations", String.format("%.1f %.1f %.1f",moveForwardShortDuration, turnLeftDuration, moveForwardLongDuration));
        //telemetry.addData("Sensor", String.format("light: %.3f", opticalDistanceSensor.getLightDetected()));
    }
}
