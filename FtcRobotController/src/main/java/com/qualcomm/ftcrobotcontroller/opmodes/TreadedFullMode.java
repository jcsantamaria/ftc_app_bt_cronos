package com.qualcomm.ftcrobotcontroller.opmodes;

public class TreadedFullMode extends TreadedBotHardware {

    @Override
    public void start() {
        //Set gripper to open
        openGripper();
    }

    @Override
    public void loop() {

        //Get the values from the gamepads
        //Note: pushing the stick all the way up returns -1,
        // so we need to reverse the y values
        float xValue = -gamepad1.left_stick_x;
        float yValue = -gamepad1.left_stick_y;
        float lValue = -gamepad1.right_stick_y;

        lValue =(float)scaleInput(lValue) * 0.3f;

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
        float magnitude = 0.75f;
        setDrivePower( magnitude * scaleInput(leftPower), magnitude * scaleInput(rightPower));

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

