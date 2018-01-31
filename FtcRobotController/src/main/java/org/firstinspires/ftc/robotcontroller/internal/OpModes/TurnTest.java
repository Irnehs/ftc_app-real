package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by kevinrockwell on 1/23/18.
 */
@Autonomous(name="Test Turn", group="Relic Recovery")
public class TurnTest extends RelicBaseAuto {


    /* Declare OpMode members. */

        RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
        String Version = "0.0.3";
        VuforiaLocalizer vuforia;


        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            waitForStart();
            //robot.colorSensor.
            turnCounterClockwise(0.5, 90, robot);
            }
        }

