package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Base class for the op-modes for the Clock robot (FTC team 9785 / Cronos).
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 */
public class WheeledBotHardware extends OpMode {

    final double RAD2DEG = 180.0 / Math.PI;
    final double DEG2RAD = Math.PI / 180.0;
    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftRearMotor;
    DcMotor leftFrontMotor;
    DcMotor rightRearMotor;
    DcMotor rightFrontMotor;
    DcMotor armMotor;
    Servo leftGrip;
    Servo rightGrip;
    GyroSensor gyroSensor;
    TouchSensor armTouch;
    OpticalDistanceSensor opticalDistanceSensor;

    private int prevLeftRearStep;
    private int prevLeftFrontStep;
    private int prevRightRearStep;
    private int prevRightFrontStep;
    
    /**
     * Absolute position of the robot in the x-axis.
     */
    public double positionX;
    /**
     * Absolute position of the robot in the y-axis.
     */
    public double positionY;

    /**
     * Absolute heading (counter-clockwise) of the robot in radians.
     */
    public double heading;

    /**
     * Indicates that location encoders is under reset.
     */
    public boolean onArmReset;


    @Override
    public void init() {

        StringBuilder sb = new StringBuilder();

        sb.append("lr_drive: ");
        try {
            leftRearMotor = hardwareMap.dcMotor.get("lr_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("lf_drive: ");
        try {
            leftFrontMotor = hardwareMap.dcMotor.get("lf_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rr_drive: ");
        try {
            rightRearMotor = hardwareMap.dcMotor.get("rr_drive");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("rf_drive: ");
        try {
            rightFrontMotor = hardwareMap.dcMotor.get("rf_drive");
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
        sb.append("arm_touch: ");
        try{
            armTouch = hardwareMap.touchSensor.get("arm_touch");
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
        sb.append("gyro ");
        try {
            gyroSensor = hardwareMap.gyroSensor.get("gyro");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }
        sb.append("optical ");
        try {
            opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("optical");
            sb.append("OK ");
        }
        catch (Exception ex) {
            sb.append("ERR ");
        }

        //Reverse the right-side motors
        if (leftRearMotor != null)
            leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        if (leftFrontMotor != null)
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set gripper to close
        closeGripper();

        //Prepare drive
        resetDriveEncoders();

        //Prepare arm
        resetArmEncoders();

        //Report status
        telemetry.addData("Status", sb.toString());
    }

    @Override
    public void loop() {
        // on arm reset, keep changing drive mode until ready
        DcMotorController.RunMode mode = armMotor.getMode();
        if ( onArmReset ) {
            // force a reset until we detect a position==0
            if ( mode != DcMotorController.RunMode.RESET_ENCODERS || armMotor.getCurrentPosition() != 0)
                armMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

            // signal reset done when ready
            if ( mode == DcMotorController.RunMode.RESET_ENCODERS && armMotor.getCurrentPosition() == 0) {
                onArmReset = false;
            }
        }
        else if (mode != DcMotorController.RunMode.RUN_WITHOUT_ENCODERS) {
            // force a power mode until we detect it
            armMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }

        // update absolution position
        updatePosition();
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
     * Move the arm with the specified power: positive value raises the arm.
     * @param power     the power level
     */
    void moveArm(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1,1);

        if ( armMotor != null && armMotor.getMode() == DcMotorController.RunMode.RUN_WITHOUT_ENCODERS ) {
            boolean atLowerLimit = armTouch != null && armTouch.isPressed();
            boolean atUpperLimit = armMotor.getCurrentPosition() > 2000;

            if ( atLowerLimit && power < 0 && !onArmReset ) {
                stopArm();
                resetArmEncoders();

                // we are done
                return;
            }


            // do not keep raising the arm if at upper limit
            if ( atUpperLimit && power > 0)
                power = 0;

            // do not keep lowering the arm if at lower limit
            if ( atLowerLimit && power < 0)
                power = 0;

            // ok send the power level
            armMotor.setPower(power);
        }
    }


    /**
     * Stop arm movement.
     */
    void stopArm() {
        if ( armMotor != null ) {
            armMotor.setPower(0);
        }
    }

    /**
     * Reset arm motor encoders.
     */
    void resetArmEncoders() {
        if ( armMotor != null ) {
            // stop motor
            armMotor.setPower(0);

            // send command
            armMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

            // we are now in reset mode
            onArmReset = true;
        }
    }

    /**
     * Set the arm motor to the specified mode.
     *
     * @param mode  the motor run mode
     */
    void setArmMode(DcMotorController.RunMode mode) {
        if ( armMotor != null )
            armMotor.setMode(mode);
    }


    /**
     * Reset drive motor encoders.
     */
    void resetDriveEncoders() {
        if ( leftRearMotor != null)
            leftRearMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (leftFrontMotor != null)
            leftFrontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (rightRearMotor != null)
            rightRearMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        if (rightFrontMotor != null)
            rightFrontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        if ( gyroSensor != null)
            gyroSensor.resetZAxisIntegrator();

        // reset relative variables
        prevLeftRearStep = 0;
        prevLeftFrontStep = 0;
        prevRightRearStep = 0;
        prevRightFrontStep = 0;

        // reset absolute variables
        positionX = 0.0;
        positionY = 0.0;
        heading   = 0.0;
    }

    /**
     * Use the motor encoders and the gyro sensor to update the absolute position of the robot.
     */
    private void updatePosition()
    {
        double distance = 0;
        int motors = 0;
        int leftRearStep = 0;
        int leftFrontStep = 0;
        int rightRearStep = 0;
        int rightFrontStep = 0;
        if ( leftRearMotor != null) {
            leftRearStep = leftRearMotor.getCurrentPosition();
            distance +=  leftRearStep - prevLeftRearStep;
            motors += 1;
        }
        if ( leftFrontMotor != null) {
            leftFrontStep = leftFrontMotor.getCurrentPosition();
            distance +=  leftFrontStep - prevLeftFrontStep;
            motors += 1;
        }
        if ( rightRearMotor != null) {
            rightRearStep = rightRearMotor.getCurrentPosition();
            distance +=  rightRearStep - prevRightRearStep;
            motors += 1;
        }
        if ( rightFrontMotor != null) {
            rightFrontStep = rightFrontMotor.getCurrentPosition();
            distance +=  rightFrontStep - prevRightFrontStep;
            motors += 1;
        }

        if ( motors > 0 && gyroSensor != null ) {
            // compute average of distance
            distance = distance / motors;

            // read angle
            int raw = gyroSensor.getHeading();
            heading = raw * DEG2RAD;

            //telemetry.addData("raw", String.format("%d %.0f", raw, distance));

            // compute displacement
            double dx = distance * Math.sin(heading);
            double dy = distance * Math.cos(heading);

            // update position
            positionX = positionX + dx;
            positionY = positionY + dy;
        }

        // prepare for next reading
        prevLeftRearStep = leftRearStep;
        prevLeftFrontStep = leftFrontStep;
        prevRightRearStep = rightRearStep;
        prevRightFrontStep = rightFrontStep;

        //telemetry.addData("front", String.format("%d %d", -leftFrontStep, -rightFrontStep));
        //telemetry.addData("rear", String.format("%d %d", -leftRearStep, -rightRearStep));
    }
    
    /**
     * Set the drive motors to the specified mode.
     *
     * @param mode  the motor run mode
     */
    void setDriveMode(DcMotorController.RunMode mode) {
        if ( leftRearMotor != null )
            leftRearMotor.setMode(mode);
        if ( leftFrontMotor != null )
            leftFrontMotor.setMode(mode);
        if ( rightRearMotor != null)
            rightRearMotor.setMode(mode);
        if ( rightFrontMotor != null )
            rightFrontMotor.setMode(mode);
    }

    /**
     * Set the power for both drive motors using the specified value.
     *
     * @param power     the power level
     */
    void setDrivePower(double power) {
        //Clip the power values so that it only goes from -1 to 1
        power = Range.clip(power, -1,1);

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && power < 0;

        if ( !stop ) {
            if (leftRearMotor != null)
                leftRearMotor.setPower(power);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(power);
            if (rightRearMotor != null)
                rightRearMotor.setPower(power);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(power);
        }
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

        // stop moving when distance sensor says we are close to an obstacle and we want to move forward
        boolean stop = opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && (leftPower < 0 || rightPower < 0);

        if ( !stop ) {
            if (leftRearMotor != null)
                leftRearMotor.setPower(leftPower);
            if (leftFrontMotor != null)
                leftFrontMotor.setPower(leftPower);
            if (rightRearMotor != null)
                rightRearMotor.setPower(rightPower);
            if (rightFrontMotor != null)
                rightFrontMotor.setPower(rightPower);
        }
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
