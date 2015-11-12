package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Student on 9/24/2015.
 */
public class SingleMotor extends OpMode
{
    //var
    Servo leMayMay;

    //init...runs 1 time only
    @Override
    public void init()
    {
        telemetry.addData("Hello init: ", "World init!");
        leMayMay = hardwareMap.servo.get("teMayMay");
    }


    //loop...runs continually
    @Override
    public void loop()
    {
        //Get Gamepad Value, make sure is negative due to forward being -1
        double yValue = -gamepad1.left_stick_y;

        //Clip the values between -1 and 1 so that it does not overpower the servo
        yValue = Range.clip(yValue, 0, 1);

        //set the servo position to the y value, which is the negative value of the gamepad
        leMayMay.setPosition(yValue);

        telemetry.addData("yValue: ", yValue);
        telemetry.addData("Hello: ", "World loop!");
    }

}
