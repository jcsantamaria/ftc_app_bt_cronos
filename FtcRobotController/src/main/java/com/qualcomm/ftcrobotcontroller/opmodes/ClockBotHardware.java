package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Base class for the op-modes for the Clock robot (FTC team 9785 / Cronos).
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 */
public class ClockBotHardware extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;

    @Override
    public void init() {

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

    }
}
