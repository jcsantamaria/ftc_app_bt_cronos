package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

enum RobotState {
        MoveForward,
        TurnLeft
}

/**
 * Simple autonomous behavior that moves the robot in a rectangle.
 */
public class AutoMode extends OpMode{

    final double FORWARD_SPEED = 0.30;
    final double TURN_SPEED = 0.15;
    final long MOVEFORWARD_DURATION = 4000;
    final long TURNLEFT_DURATION    = 2000;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;

    RobotState state;
    long timestamp;
    long moveForwardDuration = MOVEFORWARD_DURATION;
    long turnLeftDuration    = TURNLEFT_DURATION;

    @Override
    public void init() {
        //Get references to the motors and servos from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_drive");
        //leftGrip = hardwareMap.servo.get("left_grip");
        //rightGrip = hardwareMap.servo.get("right_grip");

        //Reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set running mode
        leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // initialze the state and timestamp
        moveForwardDuration = 1140;
        turnLeftDuration    = 1520;

        state = RobotState.MoveForward;
        timestamp = -1;
    }

    @Override
    public void loop() {

        switch ( state )
        {
            case MoveForward:
                {
                    // set the timestamp the first time we get in this state
                    if ( timestamp < 0 )
                        timestamp = System.currentTimeMillis();

                    // set  motors to move forward
                    leftMotor.setPower(FORWARD_SPEED);
                    rightMotor.setPower(FORWARD_SPEED);

                    // check condition to switch
                    long elapsed = System.currentTimeMillis() - timestamp;
                    if ( elapsed > moveForwardDuration)
                    {
                        state = RobotState.TurnLeft;
                        timestamp = -1;
                    }
                }
                break;

            case TurnLeft:
                {

                    // set the timestamp the first time we get in this state
                    if ( timestamp < 0 )
                        timestamp = System.currentTimeMillis();

                    // set motors to turn left
                    leftMotor.setPower(-TURN_SPEED);
                    rightMotor.setPower(TURN_SPEED);

                    //check condition to switch
                    long elapsed = System.currentTimeMillis() - timestamp;
                    if ( elapsed > turnLeftDuration)
                    {
                        state = RobotState.MoveForward;
                        timestamp = -1;
                    }
                }
                break;
        }

        // adjust the move forward duration using the up/down dpad
        if ( gamepad1.dpad_down ) {
            moveForwardDuration = Math.max(moveForwardDuration - 10, 100);
        }
        if ( gamepad1.dpad_up ) {
            moveForwardDuration = Math.min(moveForwardDuration + 10, MOVEFORWARD_DURATION);
        }

        // adjust the turn left duration using the left/right dpad
        if ( gamepad1.dpad_left ) {
            turnLeftDuration = Math.max(turnLeftDuration - 10, 100);
        }
        if ( gamepad1.dpad_right ) {
            turnLeftDuration = Math.min(turnLeftDuration + 10, TURNLEFT_DURATION);
        }

        long elapsed = System.currentTimeMillis() - timestamp;
        telemetry.addData("State", state.toString() + "  elapsed: " + elapsed);
        telemetry.addData("Forward", moveForwardDuration);
        telemetry.addData("Turn", turnLeftDuration);
    }

}
