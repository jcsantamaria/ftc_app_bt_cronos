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
        float lValue = -gamepad1.right_stick_y;

        lValue = Range.clip((float)scaleInput(lValue), -0.3f, 0.3f);

        // arm control
        if(Math.abs(lValue) > 0.05) {
            armMotor.setPower(lValue);
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

