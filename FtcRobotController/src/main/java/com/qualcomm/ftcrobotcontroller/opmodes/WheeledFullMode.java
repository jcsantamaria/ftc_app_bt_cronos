package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;

public class WheeledFullMode extends WheeledBotHardware {

    @Override
    public void start() {

        // do what our parent says first
        super.start();

        //Set drive mode
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Set drive mode
        setArmMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Set gripper to open
        openGripper();

        telemetry.addData("joy", String.format("%.2f %.2f", 0.0, 0.0));
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        telemetry.addData("rot", String.format("p:%3.0f r:%3.0f h:%3.0f", orientation.getPitch(), orientation.getRoll(), orientation.getHeading()));
    }

    @Override
    public void loop() {

        // do what our parent says first
        super.loop();

        //Get the values from the gamepads
        //Note: pushing the stick all the way up returns -1,
        // so we need to reverse the y values
        float xValue =  gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;
        float lValue = -gamepad1.right_stick_y;

        lValue = (float) scaleInput(lValue) * 0.3f;

        // arm control
        if (Math.abs(lValue) > 0.05) {
            float armMagnitude = lValue > 0 ? 0.4f : 0.3f;
            moveArm( armMagnitude * lValue);
        } else {
            stopArm();
        }

        //Calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //Set the power of the motors with the gamepad values
        float driveMagnitude = 0.5f;
        setDrivePower(driveMagnitude * scaleInput(leftPower), driveMagnitude * scaleInput(rightPower));

        // gripper control
        // This code will open and close the gripper with two buttons
        // using 1 button to open and another to close the gripper
        if (gamepad1.x) {
            openGripper();
        }
        if (gamepad1.a) {
            closeGripper();
        }

        telemetry.addData("joy", String.format("%.2f %.2f",  xValue, yValue));
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        telemetry.addData("rot", String.format("p:%3.0f r:%3.0f h:%3.0f", orientation.getPitch(), orientation.getRoll(), orientation.getHeading()));
        //telemetry.addData("touch", armTouch != null ? armTouch.isPressed() : "null");
        telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getMode().toString()));
        //telemetry.addData("distance", String.format("%.2f", opticalDistanceSensor.getLightDetected()));
    }
}

