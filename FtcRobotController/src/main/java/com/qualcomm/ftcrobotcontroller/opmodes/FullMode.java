package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class FullMode extends ClockBotHardware {

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

        //Set the power of the motors with the gamepad values
        setDrivePower( scaleInput(leftPower), scaleInput(rightPower));

        // gripper control
        // This code will open and close the gripper with two buttons
        // using 1 button to open and another to close the gripper
        if(gamepad1.x) {
            openGripper();
        }
        if(gamepad1.a) {
            closeGripper();
        }
    }

}

