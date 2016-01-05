package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * OpMode to park on the mountain.
 */
public class LowMode extends TreadedBotHardware {

    enum RobotState {
        MoveForward,
        TurnLeft,
        MoveForwardTwo,
        Stop
    }

    final double FORWARD_SPEED = 0.30;
    final double TURN_SPEED = 0.15;
    final int MOVEFORWARD_DURATION = 1100;
    final int MOVEFORWARD_DURATION_TWO = 1500;
    final int TURNLEFT_DURATION = 500;
    final double SQUELCH_DURATION = 0.5;

    RobotState state;
    StopWatch stopWatch;
    int moveForwardPosition;
    int turnLeftPosition;
    int moveForwardPositionTwo;
    StopWatch squelch;

    public void start() {
        // initialze the state and timestamp
        moveForwardPosition    = MOVEFORWARD_DURATION;
        turnLeftPosition       = TURNLEFT_DURATION;
        moveForwardPositionTwo = MOVEFORWARD_DURATION_TWO;

        state = RobotState.MoveForward;
        stopWatch = new StopWatch();
        squelch = new StopWatch();
    }

    @Override
    public void init_loop() {
        // adjust the move forward duration using the up/down dpad
        if (gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardPosition = Math.max(moveForwardPosition - 10, 100);
            squelch.reset();
        }
        if (gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
            moveForwardPosition = Math.min(moveForwardPosition + 10, 2 * MOVEFORWARD_DURATION);
            squelch.reset();
        }

        // adjust the turn left duration using the left/right dpad
        if (gamepad1.dpad_left && squelch.elapsedTime() > SQUELCH_DURATION) {
            turnLeftPosition = Math.max(turnLeftPosition - 10, 100);
            squelch.reset();
        }
        if (gamepad1.dpad_right && squelch.elapsedTime() > SQUELCH_DURATION) {
            turnLeftPosition = Math.min(turnLeftPosition + 10, 2 * TURNLEFT_DURATION);
            squelch.reset();
        }

        telemetry.addData("Forward", moveForwardPosition);
        telemetry.addData("Turn", turnLeftPosition);
        telemetry.addData("ForwardTwo", moveForwardPositionTwo);
    }

    @Override
    public void loop() {

        switch (state) {
            case MoveForward: {

                // set  motors to move forward
                setDrivePower(FORWARD_SPEED);

                // check condition to switch
                int pos = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

                // read encoders
                if (pos > moveForwardPosition) {
                    resetEncoders();
                    state = RobotState.TurnLeft;
                    stopWatch.reset();
                }
            }

            case TurnLeft: {
                // set motors to turn left
                setDrivePower(-TURN_SPEED, TURN_SPEED);

                //check condition to switch
                int pos = (-leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
                if (pos > turnLeftPosition) {
                    resetEncoders();
                    state = RobotState.MoveForwardTwo;
                    stopWatch.reset();
                }
            }
            break;

            case MoveForwardTwo:
            {
                    // set  motors to move forward
                    setDrivePower(FORWARD_SPEED);

                    // check condition to switch
                    int pos = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

                    // read encoders
                    if (pos > moveForwardPositionTwo) {
                        resetEncoders();
                        state = RobotState.Stop;
                        stopWatch.reset();
                    }
            }
            break;

            case Stop:
            {

                setDrivePower(0,0);
            }
            break;
        }

        telemetry.addData("State", String.format("%s  elapsed: %.3f", state, stopWatch.elapsedTime()));
    }
}
