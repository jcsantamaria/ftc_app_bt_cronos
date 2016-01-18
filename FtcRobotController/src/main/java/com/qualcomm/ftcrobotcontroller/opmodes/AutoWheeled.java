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
    final double DRIVE_POWER = 0.08;

    enum RobotState
    {
        LowerArm,
        RaiseArm,

        Waypoint1,
        Waypoint2,
        Stop
    }

    double targetx1 = 0;
    double targety1 = 2200;
    double targetx2 = 2200;
    double targety2 = 2200;

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
                // compute delta vector
                double dx = targetx1 - positionX;
                double dy = targety1 - positionY;

                // normalize
                double norm = Math.sqrt(dx * dx + dy * dy);
                dx = dx / norm;
                dy = dy / norm;

                //Calculate the power needed for each motor
                double leftPower  = dy + dx;
                double rightPower = dy - dx;

                //set value to be less so it's not going crazy
                double magnitude = DRIVE_POWER;
                setDrivePower(magnitude * leftPower, magnitude * rightPower);

               if (norm < 100) {
                    state = RobotState.Waypoint2;
               }
            }
            break;

            case Waypoint2:
            {
                // compute delta vector
                double dx = targetx1 - positionX;
                double dy = targety1 - positionY;

                // normalize
                double norm = Math.sqrt(dx * dx + dy * dy);
                dx = dx / norm;
                dy = dy / norm;

                //Calculate the power needed for each motor
                double leftPower  = dy + dx;
                double rightPower = dy - dx;

                //set value to be less so it's not going crazy
                double magnitude = DRIVE_POWER;
                setDrivePower(magnitude * leftPower, magnitude * rightPower);

                if (norm < 100) {
                    state = RobotState.Stop;
                }
            }
            break;

            case Stop:
            {
                setDrivePower(0);
                openGripper();
            }
            break;
        }

        telemetry.addData("state", state.toString());
        telemetry.addData("pos", String.format("%.0f %.0f", positionX, positionY));
        telemetry.addData("arm", String.format("%s %.2f %d %s", onArmReset, armMotor.getPower(), armMotor.getCurrentPosition(), armMotor.getMode().toString()));
    }
}
