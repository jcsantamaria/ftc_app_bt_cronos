package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to autonomously park the robot.
 */
public class AutoWheeled extends WheeledBotHardware {
    final double ARM_LOWER_POWER =-0.02;
    final double ARM_RAISE_POWER = 0.08;
    final int    ARM_SET_POSITION = 820;
    final double DRIVE_POWER = 0.12;
    final double WAYPOINT_THRESHOLD = 300;
    final double SQUELCH_DURATION = 0.5;
    enum RobotState
    {
        LowerArm,
        RaiseArm,

        Waypoint1,
        Waypoint2,
        Stop
    }

    double targetx1 = -2500;
    double targety1 = 4200;
    double targetx2 = -5000;
    double targety2 = 5000;
    double drive_power = DRIVE_POWER;
    RobotState state;
    StopWatch squelch;

    @Override
    public void start() {

        // do what our parent says first
        super.start();

        //Set drive mode
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Set drive mode
        setArmMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        state = RobotState.LowerArm;
        squelch = new StopWatch();

        telemetry.addData("state", state.toString());
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, heading * RAD2DEG));
    }


    @Override
    public void loop() {

        // do what our parent says first
        super.loop();

        if ( gamepad1.a ) {
            state = RobotState.Stop;
        }
        if ( state == RobotState.Stop && gamepad1.x) {
            state = RobotState.Waypoint1;
        }

        switch ( state )
        {
            case LowerArm:
            {
                moveArm(ARM_LOWER_POWER);

                if ( onArmReset ) {
                    state = RobotState.RaiseArm;
                }
            }
            break;

            case RaiseArm:
            {
                if ( armMotor.getCurrentPosition() < ARM_SET_POSITION ) {
                    moveArm(ARM_RAISE_POWER);
                }
                else {
                    stopArm();
                    state = RobotState.Waypoint1;
                }
            }
            break;

            case Waypoint1:
            {
               if (driveToTarget(targetx1, targety1, WAYPOINT_THRESHOLD)) {
                    state = RobotState.Waypoint2;
               }
            }
            break;

            case Waypoint2:
            {
                if (driveToTarget(targetx2, targety2, WAYPOINT_THRESHOLD)) {
                    state = RobotState.Stop;
                }
            }
            break;

            case Stop:
            {
                // release the climbers!
                setDrivePower(0);
                openGripper();
            }
            break;
        }


        // adjust the move forward duration using the up/down dpad
        if ( gamepad1.dpad_down && squelch.elapsedTime() > SQUELCH_DURATION) {
            drive_power = Math.max(drive_power - 0.01, 0.1);
            squelch.reset();
        }
        if ( gamepad1.dpad_up && squelch.elapsedTime() > SQUELCH_DURATION) {
            drive_power = Math.min(drive_power + 0.01, 2 * DRIVE_POWER);
            squelch.reset();
        }
        telemetry.addData("state", state.toString());
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, heading * RAD2DEG));
        //telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getMode().toString()));
    }

    /**
     * Send drive control commands to move the robot to the specified target point.
     * @param targetx       the x coordinate
     * @param targety       the y coordinate
     * @param threshold     the radius of the goal
     * @return true if robot reached target; false otherwise
     */
    boolean driveToTarget(double targetx, double targety, double threshold) {

        // compute delta vector in absolute coordinates
        double dx = targetx - positionX;
        double dy = targety - positionY;

        // normalize
        double norm = Math.sqrt(dx * dx + dy * dy);
        double angle = Math.atan2(dx,dy);

        double mag = 1;
        if ( norm < threshold * 2) {
            mag = norm / (threshold * 2);
        }

        // compute command in egocentric coordinates
        dx = mag * Math.sin(angle - heading);
        dy = mag * Math.cos(angle - heading);

        // correct control commands for reverse
        if ( dy < 0)
            dx = -dx;

        //Calculate the power needed for each motor
        double leftPower  = dy + dx;
        double rightPower = dy - dx;

        //set value to be less so it's not going crazy
        double magnitude = drive_power;
        setDrivePower(magnitude * leftPower, magnitude * rightPower);

        telemetry.addData("control", String.format("h:%3.0f a:%3.0f a-h:%3.0f dx:%4.2f dy:%4.2f p:%.2f", heading * RAD2DEG, angle * RAD2DEG, (angle - heading) * RAD2DEG, dx, dy, drive_power));

        return norm < threshold;
    }
}
