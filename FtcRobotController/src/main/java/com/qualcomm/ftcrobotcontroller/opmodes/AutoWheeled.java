package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to autonomously park the robot.
 */
public class AutoWheeled extends WheeledBotHardware {
    final double ARM_LOWER_POWER     =-0.02;
    final double ARM_RAISE_POWER     = 0.1;
    final int    ARM_DROP_POSITION   = 1700;
    final int    ARM_BEACON_POSITION = 1150;
    final double DRIVE_POWER         = 0.25;
    final double SQUELCH_DURATION    = 0.5;

    public enum RobotState
    {
        LowerArm,
        RaiseArm,

        Waypoint1Aux,       // auxiliary target
        WaypointBeacon,     // beacon target
        WaitDrop,           // wait 3 seconds and drop climbers
        BackABit,           // back up to approach beacon
        ApproachBeacon,           // approach beacon
        Stop                // stop the robot
    }

    // parameters for waypoint auxiliary
    double targetxAux           = -2800;
    double targetyAux           =  7000;
    double targetHeadingAux     =     0;
    double ApproachThresholdAux =   600;
    double WaypointThresholdAux =   350;

    // parameters for waypoint beacon
    public double targetxBeacon           = -6700;
    public double targetyBeacon           =  7150;
    public double targetHeadingBeacon     =   270;

    // debug values
//    double targetxBeacon           = 0;
//    double targetyBeacon           = 2200;
//    double targetHeadingBeacon     = 0;

    public double ApproachThresholdBeacon =  7200;
    public double WaypointThresholdBeacon =   350;

    // parameters for wait drop
    double WaitDropDuration = 3;

    // parameters for stop
    double WaitBeaconPushDuration = 1;

    public double drive_power = DRIVE_POWER;
    public RobotState state;
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
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f o:%.2f", positionX, positionY, Math.toDegrees(heading), opticalDistanceSensor.getLightDetected()));
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
            state = RobotState.Waypoint1Aux;
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
                if ( armMotor.getCurrentPosition() < ARM_DROP_POSITION) {
                    moveArm(ARM_RAISE_POWER);
                }
                else {
                    stopArm();
                    state = RobotState.WaypointBeacon;
                }
            }
            break;

            case Waypoint1Aux:
            {
                // keep moving towards target until we are within the target threshold
               if (moveToTarget(targetxAux, targetyAux, ApproachThresholdAux, WaypointThresholdAux)) {
                   // next state
                    state = RobotState.WaypointBeacon;
               }
            }
            break;

            case WaypointBeacon: {

                // compute distance to target
                double dx   = targetxBeacon - positionX;
                double dy   = targetyBeacon - positionY;
                double norm = Math.sqrt(dx * dx + dy * dy);

                // check if we reach the wall: optical sensor triggers and we are close to the wall
                boolean wallReached = opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2 && norm < 2200;

                // keep moving towards target until we are within the target threshold
                if (moveToTargetAndHeading(targetxBeacon, targetyBeacon, targetHeadingBeacon, ApproachThresholdBeacon, WaypointThresholdBeacon) || wallReached) {
                    // restart stopwatch
                    stopWatch.reset();
                    // next state
                    state = RobotState.WaitDrop;
                }
            }
            break;

            case WaitDrop:
            {
                // stop the robot
                setDrivePower(0);

                if ( stopWatch.elapsedTime() > WaitDropDuration) {
                    openGripper();

                    // set auxiliary target: 1 tile behind beacon
                    targetxAux       = targetxBeacon - 2200 * Math.sin(Math.toRadians(targetHeadingBeacon));
                    targetyAux       = targetyBeacon - 2200 * Math.cos( Math.toRadians(targetHeadingBeacon));
                    targetHeadingAux = targetHeadingBeacon;

                    //drive_power = 0;

                    // next state
                    state = RobotState.Stop;
                }
            }
            break;

            case BackABit:
            {
                //openGripper();

                // keep moving towards target until we are within the target threshold
                if (moveToTargetAndHeading(targetxAux, targetyAux, targetHeadingAux, ApproachThresholdAux, WaypointThresholdAux)) {

                    // set auxiliary target: 1 1/2 tiles in beyond beacon
                    targetxAux       = targetxBeacon + 3300 * Math.sin(Math.toRadians(targetHeadingBeacon));
                    targetyAux       = targetyBeacon + 3300 * Math.cos(Math.toRadians(targetHeadingBeacon));
                    targetHeadingAux = targetHeadingBeacon;

                    // next state
                    state = RobotState.ApproachBeacon;
                }
            }
            break;

            case ApproachBeacon:
            {
                if ( armMotor.getCurrentPosition() > ARM_BEACON_POSITION) {
                    moveArm(ARM_LOWER_POWER);
                }
                else {
                    stopArm();
                }

                // check if we reach the wall: optical sensor triggers and we are close to the wall
                boolean wallReached = (opticalDistanceSensor != null && opticalDistanceSensor.getLightDetected() > 0.2) ||
                                      (beaconTouch != null && beaconTouch.isPressed());

                // keep moving towards target until we are within the target threshold
                if (moveToTargetAndHeading(targetxAux, targetyAux, targetHeadingAux, ApproachThresholdAux, WaypointThresholdAux) || wallReached) {
                    // restart stopwatch
                    stopWatch.reset();
                    // next state
                    state = RobotState.Stop;
                }
            }
            break;

            case Stop:
            {
                // release the climbers!
                setDrivePower(0);
                closeGripper();

                if ( stopWatch.elapsedTime() > WaitBeaconPushDuration ) {
                    //pushRightGripper();
                }
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
        telemetry.addData("pos", String.format("x:%4.0f y:%4.0f h:%3.0f o:%.2f", positionX, positionY, Math.toDegrees(heading), opticalDistanceSensor.getLightDetected()));
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
        final double MAX_DELTA_HEADING = Math.toRadians(10);

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
        else if ( Math.abs(onX) > targetThreshold / 2)
        {
            // robot is within approach threshold: lets move towards target along an ellipse
            double theta = onX > 0 ? Math.atan2(onY, onX - approachThreshold) : Math.atan2(onY, onX + approachThreshold);
            double b = onY / Math.sin(theta);    // mayor radius
            double a = approachThreshold;        // minor radius
            double r = (a + b) / 2f;             // average radius

            // heading based on equation of an ellipse
            // but the ellipse depends if the robot is on the positive side or negative size along x axis
            double onh = onX > 0 ? Math.atan2( a * Math.sin(theta), -b * Math.cos(theta) ) :
                                   Math.atan2(-a * Math.sin(theta),  b * Math.cos(theta) );
            // convert to global frame
            onh = NormalizeAngle(onh + targetHeading, -Math.PI);

            // turn bias based on average radius
            dx = Range.clip(Math.signum(onX) * 500 / r, -0.5, 0.5);

            // compute the error between the desired and current heading
            double deltaHeading = NormalizeAngle(onh - heading, -Math.PI);

            // extra turn to correct the heading error
            dx = Range.clip( dx + deltaHeading / MAX_DELTA_HEADING, -1.0, 1.0);

            // full speed unless we are getting close
            double mag = Range.clip(norm / targetThreshold, 0.0, 1.0);

            // forward depends on how much we steer
            dy = mag * (1.0 - Math.abs(dx));
        }
        else {

            // compute the error between the desired and current heading
            double deltaHeading = NormalizeAngle(targetHeading - heading, -Math.PI);

            // correct the heading error
            dx = Range.clip(-Math.signum(onY) * deltaHeading / (2.0 * MAX_DELTA_HEADING), -1.0, 1.0);

            // full speed unless we are getting close
            double mag = -Math.signum(onY) * 0.3 * Range.clip( norm / targetThreshold, 0.0, 1.0);

            // forward depends on how much we steer
            dy = mag * (1.0 - 0.75 * Math.abs(dx));
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
