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
public class EncoderCalibrate extends OpMode {
    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;
    int targetPosition;
    boolean engage;

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

        // reset encoders
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        // set default target position
        targetPosition = 500;
        engage = false;

        telemetry.addData("Encode", targetPosition);
        telemetry.addData("Engage", engage);
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

            //leftMotor.setTargetPosition(targetPosition);
            //rightMotor.setTargetPosition(targetPosition);
            leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            leftMotor.setPower(0.1);
            rightMotor.setPower(0.1);
            engage = true;
        }
        else if (engage)
        {
            int leftpos = leftMotor.getCurrentPosition();
            int rightpos = rightMotor.getCurrentPosition();
            if ( leftpos > targetPosition || rightpos > targetPosition)
            {
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                engage = false;
            }
        }

        telemetry.addData("Encode", targetPosition);
        telemetry.addData("Engage", engage);
        telemetry.addData("Position", leftMotor.getCurrentPosition() + " " +  rightMotor.getCurrentPosition());
     }

}

