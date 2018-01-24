package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kevinrockwell on 1/23/18.
 */
@Autonomous(name="Test Color", group="Relic Recovery")
public class ColorSensorTest extends RelicBaseAuto {


    /* Declare OpMode members. */

        //RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
        String Version = "0.0.3";
        VuforiaLocalizer vuforia;


        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            waitForStart();
            //robot.colorSensor.
            while (opModeIsActive()) {
                telemetry.addData("Red ", robot.colorSensor.getNormalizedColors().red);
                telemetry.addData("Blue ", robot.colorSensor.getNormalizedColors().blue);
                telemetry.addData("Green ", robot.colorSensor.getNormalizedColors().green);
                telemetry.update();
            }
        }
    }
