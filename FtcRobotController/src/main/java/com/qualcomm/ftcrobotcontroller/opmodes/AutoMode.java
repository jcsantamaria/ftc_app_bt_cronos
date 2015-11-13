package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

enum RobotState {
        MoveForward,
        TurnLeft
}

/**
 * Created by Student on 11/5/2015.
 */
public class Automode extends OpMode{

    final double FORWARD_SPEED = 0.50;
    final double TURN_SPEED = 0.30;
    final long MOVEFORWARD_DURATION = 2000;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;

    RobotState state;
    long timestamp;

    @Override
    public void init() {
        //Get references to the motors and servos from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_drive");
        leftGrip = hardwareMap.servo.get("left_grip");
        rightGrip = hardwareMap.servo.get("right_grip");

        // initialze the state and timestamp
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
                    if ( elapsed > MOVEFORWARD_DURATION)
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
                    leftMotor.setPower(TURN_SPEED);
                    rightMotor.setPower(TURN_SPEED);

                    //check condition to switch
                    if ()
                    {
                        long elapsed = System.currentTimeMillis() - timestamp;
                        if ( elapsed > MOVEFORWARD_DURATION)

                        state = RobotState.MoveForward
                    }
                }
                break;
        }

    }

}
