package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to calibrate gripper.
 */
public class GripCalibrate extends TreadedBotHardware {

    final double SQUELCH_DURATION = 0.5;

    double leftOpenPosition;
    double leftClosePosition;
    double rightOpenPosition;
    double rightClosePosition;
    StopWatch squelch;

    @Override
    public void start() {
        // set default target position
        leftOpenPosition = 0.0;
        leftClosePosition = 0.5;
        rightOpenPosition = 0.0;
        rightClosePosition = 0.5;

        squelch = new StopWatch();

        telemetry.addData("left", leftOpenPosition + " " + leftClosePosition);
    }

    @Override
    public void loop() {
        // arm control
        if(gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
            leftOpenPosition += 0.01;
            Range.clip(leftOpenPosition, 0, 1);
            squelch.reset();
        }
        else if(gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
            leftOpenPosition -= 0.01;
            Range.clip(leftOpenPosition, 0, 1);
            squelch.reset();
        }
        if(gamepad1.dpad_right && squelch.elapsedTime() > SQUELCH_DURATION) {
            leftClosePosition += 0.01;
            Range.clip(leftClosePosition, 0, 1);
            squelch.reset();
        }
        else if(gamepad1.dpad_left && squelch.elapsedTime() > SQUELCH_DURATION) {
            leftClosePosition -= 0.01;
            Range.clip(leftClosePosition, 0, 1);
            squelch.reset();
        }
        else if (gamepad1.y) {

            leftGrip.setPosition(leftOpenPosition);
        }
        else if (gamepad1.x)
        {
            leftGrip.setPosition(leftClosePosition);
        }
        telemetry.addData("left", leftOpenPosition + " " + leftClosePosition);
        //---------------------------------------------------------------------------------------------------
        if(gamepad1.dpad_up) {
            rightOpenPosition += 0.01;
            Range.clip(rightOpenPosition, 0, 1);
        }
        else if(gamepad1.dpad_down) {
            rightOpenPosition -= 0.01;
            Range.clip(rightOpenPosition, 0, 1);
        }
        if(gamepad1.dpad_right) {
            rightClosePosition += 0.01;
        }
        else if(gamepad1.dpad_left) {
            rightClosePosition -= 0.01;
        }
        else if (gamepad1.b) {

            rightGrip.setPosition(rightOpenPosition);
        }
        else if (gamepad1.a)
        {
            rightGrip.setPosition(rightClosePosition);
        }
        telemetry.addData("right", rightOpenPosition + " " + rightClosePosition);
    }
}


