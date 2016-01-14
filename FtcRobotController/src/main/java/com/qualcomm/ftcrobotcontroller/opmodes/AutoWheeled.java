package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * OpMode to autonomously park the robot.
 */
public class AutoWheeled extends WheeledBotHardware {
    enum RobotState
    {
        Wait,
        Waypoint1,
        Waypoint2,
        Stop
    }

    float targetx1;
    float targety1;
    float targetx2;
    float targety2;
    //OpticalDistanceSensor opticalDistanceSensor;

    RobotState state;
    StopWatch squelch;

    @Override
    public void start() {

        // initialze the state, gyro, ods and timestamp
        //opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("distance_sensor");
        //opticalDistanceSensor.enableLed(true);


        //Set drive mode
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);


        state = RobotState.Wait;
        squelch = new StopWatch();
    }


    @Override
    public void loop() {

        if ( gamepad1.a ) {
            state = RobotState.Stop;
        }
        if ( state == RobotState.Stop && gamepad1.x) {
            state = RobotState.Waypoint1;
        }

        switch ( state )
        {
            case Wait:
            {
                if (gamepad1.a) {
                    state = RobotState.Waypoint1;
                }
            }
            break;

            case Waypoint1:
            {


                float dx = targetx1 - (float)positionX;
                float dy = targety1 - (float)positionY;


                //Calculate the power needed for each motor
                float leftPower = dy + dx;
                float rightPower = dy - dx;


                //set value to be less so it's not going crazy
                float magnitude = 0.5f;
                setDrivePower(magnitude * (leftPower), magnitude * (rightPower));

               if (Math.abs(dx)<= 100 && Math.abs(dy)<= 100)
                {
                    state = RobotState.Waypoint2;
                }


            }
            break;

            case Waypoint2:
            {

                float dx1 = targetx2 - (float)positionX;
                float dy1 = targety2 - (float)positionY;


                //Calculate the power needed for each motor
                float leftPower = dy1 + dx1;
                float rightPower = dy1 - dx1;


                //set value to be less so it's not going crazy
                float magnitude = 0.5f;
                setDrivePower(magnitude * (leftPower), magnitude * (rightPower));

                if (Math.abs(dx1)<= 100 && Math.abs(dy1)<= 100)
                {
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


        //telemetry.addData("Sensor", String.format("light: %.3f", opticalDistanceSensor.getLightDetected()));
    }
}
