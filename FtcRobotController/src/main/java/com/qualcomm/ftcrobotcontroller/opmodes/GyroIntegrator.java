package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Computes absolute angles from raw inertia measurements.
 */
public class GyroIntegrator {
    double pitch;
    double roll;
    double heading;

    int prevX;
    int prevY;
    int prevZ;
    ElapsedTime stopWatch;

    /**
     * Constructs an initialized instance.
     */
    public GyroIntegrator() {
        reset();
    }

    /**
     * Update the angles using the raw readings.
     * @param rawX  raw rotation along x axis
     * @param rawY  raw rotation along y axis
     * @param rawZ  raw rotation along z axis
     */
    public void update(int rawX, int rawY, int rawZ) {
        if ( stopWatch == null ) {
            // first time?  initialize the stop watch
            stopWatch = new ElapsedTime();
        }
        else {
            // integrate values
            double elapsed = stopWatch.time();

            pitch   += ((rawX + prevX) / 2.0) * elapsed;
            roll    += ((rawY + prevY) / 2.0) * elapsed;
            heading += ((rawZ + prevZ) / 2.0) * elapsed;

            stopWatch.reset();
        }

        // remember these readings
        prevX = rawX;
        prevY = rawY;
        prevZ = rawZ;
    }

    /**
     * Resets the instance: all angles back to 0.
     */
    void reset() {
        // angles back to 0
        pitch = 0;
        roll  = 0;
        heading = 0;

        // no running stopwatch
        stopWatch = null;
    }

    /**
     * Returns the computed pitch.
     * @return  pitch
     */
    public double getPitch() {
        return pitch;
    }

    /**
     * Returns the computed roll.
     * @return  roll
     */
    public double getRoll() {
        return roll;
    }

    /**
     * Returns the computed heading.
     * @return  heading
     */
    public double getHeading() {
        return heading;
    }
}
