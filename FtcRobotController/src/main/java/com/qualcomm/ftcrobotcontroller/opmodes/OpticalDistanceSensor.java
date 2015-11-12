package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Student on 10/6/2015.
 */
public class OpticalDistanceSensor extends OpMode {
    com.qualcomm.robotcore.hardware.OpticalDistanceSensor aimbot_hax;
//Initial line
    @Override
    public void init() {
      aimbot_hax = hardwareMap.opticalDistanceSensor.get("aimbot_hax");
    }
    //Overrides init. Is looped over
    @Override
    public void loop() {

        double distance = aimbot_hax.getLightDetected();
        telemetry.addData("Distance: ", distance);
    }
}
