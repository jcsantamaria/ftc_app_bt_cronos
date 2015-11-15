package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to help calibrate target position for autonomous mode.
 */
public class EncoderCalibrate extends ClockBotHardware {
    int targetPosition;
    boolean engage;

    @Override
    public void start() {

        // set default target position
        targetPosition = 500;
        engage = false;

        telemetry.addData("Encode", targetPosition);
        telemetry.addData("Engage", engage);
        if ( leftMotor != null && rightMotor != null)
            telemetry.addData("Position", leftMotor.getCurrentPosition() + " " + rightMotor.getCurrentPosition());
    }

    @Override
    public void loop() {

        // arm control
        if(gamepad1.dpad_up) {
            targetPosition += 1;
        }
        else if(gamepad1.dpad_down) {
            targetPosition -= 1;
        }
        else if (!engage && gamepad1.y) {

            resetEncoders();
            setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            setDrivePower(0.25);

            engage = true;
        }
        else if (engage)
        {
            int leftpos = leftMotor.getCurrentPosition();
            int rightpos = rightMotor.getCurrentPosition();
            if ( leftpos > targetPosition || rightpos > targetPosition)
            {
                // stop motor
                setDrivePower(0);

                engage = false;
            }
        }

        telemetry.addData("Encode", targetPosition);
        telemetry.addData("Engage", engage);
        telemetry.addData("Position", leftMotor.getCurrentPosition() + " " +  rightMotor.getCurrentPosition());
     }

}

