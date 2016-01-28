package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to autonomously park the robot.
 */
public class AutoWheeled extends WheeledBotHardware {
    final double ARM_LOWER_POWER  =-0.02;
    final double ARM_RAISE_POWER  = 0.1;
    final int    ARM_SET_POSITION = 1700;
    final double DRIVE_POWER      = 0.20;
    final double SQUELCH_DURATION = 0.5;

    enum RobotState
    {
        LowerArm,
        RaiseArm,

        Waypoint1,
        Waypoint2,
        WaitDrop,
        Stop
    }

    // parameters for waypoint 1
    double targetx1           = -2800;
    double targety1           =  7000;
    double ApproachThreshold1 =   600;
    double WaypointThreshold1 =   400;

    // parameters for waypoint 2
    double targetx2           = -6700;
    double targety2           =  7100;
    double targetHeading2     =   270;
    double ApproachThreshold2 =  7200;
    double WaypointThreshold2 =   350;

    // parameters for wait drop
    double WaitDropDuration = 3;

    double drive_power = DRIVE_POWER;
    RobotState state;
    StopWatch  stopWatch;
    StopWatch  squelch;

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
        stopWatch = new StopWatch();

        telemetry.addData("state", state.toString());
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
    }


    @Override
    public void loop() {

        // do what our parent says first
        super.loop();

        // manual overrides for debugging
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
                    state = RobotState.Waypoint2;
                }
            }
            break;

            case Waypoint1:
            {
                // keep moving towards target until we are within the target threshold
               if (moveToTarget(targetx1, targety1, ApproachThreshold1, WaypointThreshold1)) {
                   // next state
                    state = RobotState.Waypoint2;
               }
            }
            break;

            case Waypoint2: {

                // compute distance to target
                double dx   = targetx2 - positionX;
                double dy   = targety2 - positionY;
                double norm = Math.sqrt(dx * dx + dy * dy);

                // check if we reach the wall: optical sensor triggers and we are close to the wall
                boolean wallReached = opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && norm < 2200;

                // keep moving towards target until we are within the target threshold
                if (moveToTargetAndHeading(targetx2, targety2, targetHeading2, ApproachThreshold2, WaypointThreshold2) || wallReached) {
                    // restart stopwatch
                    stopWatch.reset();
                    // next state
                    state = RobotState.WaitDrop;
                }
            }
            break;

            case WaitDrop: {

                // stop the robot
                setDrivePower(0);

                if ( stopWatch.elapsedTime() > WaitDropDuration) {
                    // next state
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
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f", positionX, positionY, Math.toDegrees(heading)));
        //telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getMode().toString()));
    }

    /**
     * Send drive control commands to move the robot to the specified target point.
     * @param targetx       the x coordinate
     * @param targety       the y coordinate
     * @param approachThreshold   the radius to slow down toward the goal
     * @param targetThreshold     the radius to reach the goal
     * @return true if robot reached target; false otherwise
     */
    boolean moveToTarget(double targetx, double targety, double approachThreshold, double targetThreshold ) {

        // compute delta vector in absolute coordinates
        double dx = targetx - positionX;
        double dy = targety - positionY;

        // compute norm and angle of delta vector
        double norm  = Math.sqrt(dx * dx + dy * dy);
        double angle = Math.atan2(dx, dy);

        double mag = 1;
        if ( norm < targetThreshold) {
            // slow down to 0 power
            mag = 0.4 * norm / targetThreshold;
        }
        else if ( norm < approachThreshold) {
            // slow down to approach speed of 0.4
            mag = 0.4 + 0.6 * (norm - targetThreshold) / (approachThreshold - targetThreshold);
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

        telemetry.addData("control", String.format("h:%3.0f a:%3.0f a-h:%3.0f dx:%4.2f dy:%4.2f p:%.2f", Math.toDegrees(heading),
                                                                                                         Math.toDegrees(angle),
                                                                                                         Math.toDegrees(angle - heading), dx, dy, drive_power));

        return norm < targetThreshold;
    }

    /**
     * Send drive control commands to move the robot to the specified target point and the specified
     * target heading.  The robot moves in along eliptical trajectories to accomplish the task.
     * @param targetX           the x coordinate
     * @param targetY           the y coordinate
     * @param targetHeadingDeg  the target heading in degrees
     * @param approachThreshold   the radius to slow down toward the goal
     * @param targetThreshold     the radius to reach the goal
     * @return true if robot reached target; false otherwise
     */
    boolean moveToTargetAndHeading(double targetX, double targetY, double targetHeadingDeg, double approachThreshold, double targetThreshold)
    {
        final double MAX_DELTA_HEADING_RAD = Math.toRadians(10);
        final double MAX_TETHA_RAD         = Math.toRadians(15);

        // use vectors for target and position
        Vector2 target   = new Vector2(targetX, targetY);
        Vector2 position = new Vector2(positionX, positionY);

        double targetHeading = Math.toRadians(targetHeadingDeg);

        // compute the delta vector: from target to position
        Vector2 delta = Vector2.subtract(position, target);
        double  norm  = delta.getMagnitude();

        // compute a coordinate system at target: x axis is horizontal and y axis is vertical
        Vector2 yaxis = new Vector2(Math.sin(targetHeading), Math.cos(targetHeading));
        Vector2 xaxis = new Vector2(yaxis.y, -yaxis.x);

        // find the projection of the delta vector on target coordinates
        double onX = Vector2.Dot(delta, xaxis);
        double onY = Vector2.Dot(delta, yaxis);

        // compute control commands
        double dx, dy;
        if ( Math.abs(onX) > approachThreshold )
        {
            // robot is too far off: lets head for the focal point
            Vector2 focal = Vector2.subtract(target, Vector2.product(approachThreshold, yaxis));
            delta = Vector2.subtract(focal, position);

            double angle = Math.atan2(delta.x, delta.y);

            dx = Math.sin(angle - heading);
            dy = Math.cos(angle - heading);
        }
        else
        {
            // robot is within approach threshold: lets move towards target along an ellipse
            double theta = onX > 0 ? Math.atan2(onY, onX - approachThreshold) : Math.atan2(onY, onX + approachThreshold);
            double b = onY / Math.sin(theta);    // mayor radius
            double a = approachThreshold;        // minor radius
            double r = (a + b) / 2f;             // average radius

            // heading based on equation of an ellipse
            // but the ellipse depends if the robot is on the positive side or negatice size alogn x axis
            double onh = onX > 0 ? Math.atan2( a * Math.sin(theta), -b * Math.cos(theta) ) :
                                   Math.atan2(-a * Math.sin(theta),  b * Math.cos(theta) );
            // convert to global frame
            onh = NormalizeAngle(onh + targetHeading, -Math.PI);

            // turn bias based on average radius
            dx = Range.clip(Math.signum(onX) * 2000 / r, -0.5, 0.5);

            // compute the error between the desired and current heading
            double deltaHeadingRad = NormalizeAngle(onh - heading, -Math.PI);

            // extra turn to correct the heading error
            dx += Range.clip(deltaHeadingRad / MAX_DELTA_HEADING_RAD, -1, 1);

            // full speed unless we are getting close
            double mag = Range.clip(norm / targetThreshold, 0, 1);

            // forward depends on how much we steer
            dy = mag * (1f - Math.abs(dx));
        }

        // correct control commands for reverse
        if (dy < 0)
            dx = -dx;

        //Calculate the power needed for each motor
        double leftPower  = dy + dx;
        double rightPower = dy - dx;

        //set value to be less so it's not going crazy
        double magnitude = drive_power;
        setDrivePower(magnitude * leftPower, magnitude * rightPower);

        telemetry.addData("control", String.format("h:%3.0f dx:%4.2f dy:%4.2f p:%.2f", Math.toDegrees(heading), dx, dy, drive_power));

        return norm < targetThreshold;
    }
}
