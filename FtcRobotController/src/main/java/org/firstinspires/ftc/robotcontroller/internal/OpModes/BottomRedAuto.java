package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kevinrockwell on 1/14/18.
 */

@Autonomous(name="Bottom Red Auto", group="Relic Recovery")
public class BottomRedAuto extends RelicBaseAuto {

    /* Declare OpMode members. */

    //RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

               /*Shows that opMode loaded correctly*/
        sayAndPause("RelicAuto: ", "Connected", 0);

        /*Drive variables*/
        int leftColumn = 44;
        int middleColumn = 36;
        int rightColumn = 28;
        int turnTicks = 800;
        double turnSpeed = 0.5;
        double straightSpeed = 0.2;

        long breakTime = 250;
        centerDistance = 41;
        /*Initialize the hardware variables with init button*/
        robot.init(hardwareMap);

        /*Vuforia Setup*/
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUAchNn/////AAAAGfqAcfY2+0TviBOpWNWvbFVO+Ki3ke54hx4bK3LAyMEOoMpSZ8pC6zWh9BQwmaUwpR8FxMbNylft5qxYuRVSaA5ijKZj2Gd5F4m8TKzk9YD+ZTRH0T/bzvhZLMr1IEnUKN0wyLqGqQqvI05qNqNahVd9OAHgy+MnrcWfrF1Ta1GUzQGc18K2qC7mioQFIJhc/KMCaFhmOer2sjtmxIp/kak0iDJfp77f/8kWvyV2IlnlR187HHWg1mgF9ZZspTYArFZa150FozF7PF7cR9xOuQZT7LuiwO/Ia64M/qa4vcOTlcHVtz6CVVC54KW1AAhQEg3p5kkG1hGbHJtvGovp7PKfragvZascLTnkCt4XK28C";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Ready to start
        sayAndPause("Ready to start", "", 0);
        waitForStart();

        //Game starts
        sayAndPause("Game starting", "", 250);

        //Starts at 1300(start + 1000) needed - 2240 +120 = 2360
        sayAndPause("Claw: ", "Closing", 500);
        closingClaw(robot);

        sayAndPause("Arm: ", "Raising", 500);
        raiseArm(robot, 2110);

        extendLeadScrew(robot);

        blueBallKnock(robot);

        //Start of vuforia
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        double vuforiaStart = getRuntime();

        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Time elapsed: ", getRuntime() - (vuforiaStart));
            telemetry.update();
            if((getRuntime() - (vuforiaStart)) > 2) {
                vuMark = RelicRecoveryVuMark.CENTER;
            }
        }
        telemetry.addData("VuMark:", vuMark);
        telemetry.update();
        sleep(250);

        driveBackward(0.5, centerDistance, robot);
        //TODO: Turn, add vuforia logic for drive time ^^, forward, place block, TEST!!!
        redVuforia(vuMark, robot);

        turnClockwise(turnSpeed, 90, 0, robot);

        placeBlock(robot, breakTime);
        /* CODE FOR THE END OF THE PROGRAM*/

        /*Turns all motors off*/
        noDrive(robot);
        robot.arm.setPower(0);
        robot.leadScrew.setPower(0);

        /*Declares end of program in telemetry*/
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();

    }
}
