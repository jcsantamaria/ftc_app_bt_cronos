package com.qualcomm.ftcrobotcontroller.opmodes;


/**
 * Simple autonomous behavior that moves the robot in a rectangle.
 */
public class AutoMode extends TreadedBotHardware {
    enum RobotState {
        MoveForward,
        TurnLeft
    }

    final double FORWARD_SPEED = 0.30;
    final double TURN_SPEED = 0.15;
    final double MOVEFORWARD_DURATION = 4;
    final double TURNLEFT_DURATION    = 2;
    final double SQUELCH_DURATION = 0.5;

    RobotState state;
    StopWatch stopWatch;
    double moveForwardDuration = MOVEFORWARD_DURATION;
    double turnLeftDuration    = TURNLEFT_DURATION;
    StopWatch squelch;

    @Override
    public void start() {

        // initialze the state and timestamp
        moveForwardDuration = 4;
        turnLeftDuration    = 2;

        state = RobotState.MoveForward;
        stopWatch = new StopWatch();
        squelch = new StopWatch();
    }

    @Override
    public void loop() {

        switch ( state )
        {
            case MoveForward:
                {
                    // set  motors to move forward
                    setDrivePower(FORWARD_SPEED);

                    // check condition to switch
                    if ( stopWatch.elapsedTime() > moveForwardDuration)
                    {
                        state = RobotState.TurnLeft;
                        stopWatch.reset();
                    }
                }
                break;

            case TurnLeft:
                {
                    // set motors to turn left
                    setDrivePower(-TURN_SPEED, TURN_SPEED);

                    //check condition to switch
                    if ( stopWatch.elapsedTime() > turnLeftDuration)
                    {
                        state = RobotState.MoveForward;
                        stopWatch.reset();
                    }
                }
                break;
        }

        // adjust the move forward duration using the up/down dpad
        if ( gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardDuration = Math.max(moveForwardDuration - 0.01, 0.1);
            squelch.reset();
        }
        if ( gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardDuration = Math.min(moveForwardDuration + 0.01, 2 * MOVEFORWARD_DURATION);
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

        telemetry.addData("State", String.format("%s  elapsed: %.3f", state, stopWatch.elapsedTime()));
        telemetry.addData("Forward", moveForwardDuration);
        telemetry.addData("Turn", turnLeftDuration);
    }

}
