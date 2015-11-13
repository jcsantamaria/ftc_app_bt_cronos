package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class FullMode extends OpMode {

    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;

    @Override
    public void init() {
        //Get references to the motors and servos from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        armMotor = hardwareMap.dcMotor.get("arm_drive");
        leftGrip = hardwareMap.servo.get("left_grip");
        rightGrip = hardwareMap.servo.get("right_grip");

        //Reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set running mode
        leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    @Override
    public void loop() {

        //Get the values from the gamepads
        //Note: pushing the stick all the way up returns -1,
        // so we need to reverse the y values
        float xValue = -gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;

        // arm control
        if(gamepad1.y) {
            armMotor.setPower(0.1);
        }
        else if(gamepad1.b) {
            armMotor.setPower(-0.1);
        }
        else {
            armMotor.setPower(0);
        }

        //Calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //Clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        //Set the power of the motors with the gamepad values
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // gripper control
        // This code will open and close the gripper with two buttons
        // using 1 button to open and another to close the gripper
        if(gamepad1.x) {
            leftGrip.setPosition(LEFT_OPEN_POSITION);
            rightGrip.setPosition(RIGHT_OPEN_POSITION);
        }
        if(gamepad1.a) {
            leftGrip.setPosition(LEFT_CLOSED_POSITION);
            rightGrip.setPosition(RIGHT_CLOSED_POSITION);
        }
    }

}

