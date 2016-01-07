package com.qualcomm.ftcrobotcontroller.opmodes;

public class WheeledFullMode extends WheeledBotHardware {

     public double _posx;
     public double _posy;



    @Override
    public void start() {
        //Set gripper to open
        openGripper();

        _posx = 0;
        _posy = 0;

    }


    @Override
    public void loop() {


        double distance = (leftRearMotor.getCurrentPosition() + leftFrontMotor.getCurrentPosition() + rightFrontMotor.getCurrentPosition() + rightRearMotor.getCurrentPosition()) / 4;
        double angle =  0; //gyroSensor.getHeading() * 3.141592657172 / 180.0;

        double dx = distance * Math.cos(angle);
        double dy = distance * Math.sin(angle);

        _posx = _posx + dx;
        _posy = _posy + dy;

        //Get the values from the gamepads
        //Note: pushing the stick all the way up returns -1,
        // so we need to reverse the y values
        float xValue = -gamepad1.left_stick_x;
        float yValue = gamepad1.left_stick_y;
        float lValue = -gamepad1.right_stick_y;

        lValue = (float) scaleInput(lValue) * 0.3f;

        // arm control
        if (Math.abs(lValue) > 0.05) {
            moveArm(lValue);
        } else {
            stopArm();
        }

        //Calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //Set the power of the motors with the gamepad values
        float magnitude = 0.75f;
        setDrivePower(magnitude * scaleInput(leftPower), magnitude * scaleInput(rightPower));

        // gripper control
        // This code will open and close the gripper with two buttons
        // using 1 button to open and another to close the gripper
        if (gamepad1.x) {
            openGripper();
        }
        if (gamepad1.a) {
            closeGripper();
        }

        telemetry.addData("posx ",_posx);
        telemetry.addData("poy ",_posy);

    }
}

