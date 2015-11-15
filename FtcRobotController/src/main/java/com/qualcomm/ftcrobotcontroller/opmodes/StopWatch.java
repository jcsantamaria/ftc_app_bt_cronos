package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 *  The <tt>Stopwatch</tt> data type is for measuring
 *  the time that elapses between the start and end of a
 *  programming task (wall-clock time).
 */


public class StopWatch {

    private long start;

    /**
     * Initializes a new stopwatch.
     */
    public StopWatch() {
        reset();
    }

    /**
     * Resets the counter.
     */
    public void reset() {
        start = System.currentTimeMillis();
    }

    /**
     * Returns the elapsed CPU time (in seconds) since the stopwatch was created.
     *
     * @return elapsed CPU time (in seconds) since the stopwatch was created
     */
    public double elapsedTime() {
        long now = System.currentTimeMillis();
        return (now - start) / 1000.0;
    }

}
