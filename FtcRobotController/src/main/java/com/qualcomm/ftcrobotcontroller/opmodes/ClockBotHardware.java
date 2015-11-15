package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Base class for the op-modes for the Clock robot (FTC team 9785 / Cronos).
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 */
public class ClockBotHardware extends OpMode {

    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;

    @Override
    public void init() {

        StringBuilder sb = new StringBuilder();

        sb.append("left_drive: ");
        try {
            leftMotor = hardwareMap.dcMotor.get("left_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        sb.append("right_drive: ");
        try {
            rightMotor = hardwareMap.dcMotor.get("right_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        sb.append("arm_drive: ");
        try{
            armMotor = hardwareMap.dcMotor.get("arm_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        sb.append("left_grip: ");
        try {
            leftGrip = hardwareMap.servo.get("left_grip");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        sb.append("right_grip: ");
        try {
            rightGrip = hardwareMap.servo.get("right_grip");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        //Reverse the right motor
        if (rightMotor != null)
            rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set gripper to close
        closeGripper();

        //Prepare drive
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        resetEncoders();

        //Report status
        telemetry.addData("Status", sb.toString());
    }

    @Override
    public void loop() {

    }

    /**
     * Set the gripper to open position.
     */
    void openGripper() {
        if (leftGrip != null)
            leftGrip.setPosition(LEFT_OPEN_POSITION);
        if (rightGrip != null)
            rightGrip.setPosition(RIGHT_OPEN_POSITION);
    }

    /**
     * Set the gripper to close position.
     */
    void closeGripper() {
        if (leftGrip != null)
            leftGrip.setPosition(LEFT_CLOSED_POSITION);
        if (rightGrip != null)
            rightGrip.setPosition(RIGHT_CLOSED_POSITION);
    }

    /**
     * Reset drive motor encoders.
     */
    void resetEncoders() {
        if ( leftMotor != null)
            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (rightMotor != null)
            rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    /**
     * Set the drive motors to the specified mode.
     */
    void setDriveMode(DcMotorController.RunMode mode) {
        if ( leftMotor != null )
            leftMotor.setMode(mode);
        if ( rightMotor != null)
            rightMotor.setMode(mode);
    }

    /**
     * Set the power for both drive motors using the specified value.
     *
     * @param power     the power level
     */
    void setDrivePower(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1,1);

        if ( leftMotor != null )
            leftMotor.setPower(power);
        if ( rightMotor != null)
            rightMotor.setPower(power);
    }

    /**
     * Set the power for both drive motors using the specified values.
     *
     * @param leftPower     the power level of the left drive motor
     * @param rightPower    the power level of the right drive motor
     */
    void setDrivePower(double leftPower, double rightPower) {
        //Clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1,1);
        rightPower = Range.clip(rightPower, -1,1);

        if ( leftMotor != null )
            leftMotor.setPower(leftPower);
        if ( rightMotor != null)
            rightMotor.setPower(rightPower);
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double value)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int)Math.round (value * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (value < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
