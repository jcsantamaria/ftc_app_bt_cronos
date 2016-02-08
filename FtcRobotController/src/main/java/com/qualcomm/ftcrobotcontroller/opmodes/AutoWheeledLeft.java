package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * AutoWheeled with target beacon on the left side.
 */
public class AutoWheeledLeft extends AutoWheeled {

    public AutoWheeledLeft() {

        // parameters for waypoint beacon
        targetxBeacon           = -6800;
        targetyBeacon           =  7150;
        targetHeadingBeacon     =   270;
    }
}
