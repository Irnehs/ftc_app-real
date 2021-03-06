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
        long leftColumnTime = 1400;
        long middleColumnTime = 1200;
        long rightColumnTime = 1000;
        long turnTime = 800;
        double turnSpeed = 0.5;
        double straightSpeed = 0.2;

        long breakTime = 250;

        /*Initialize the hardware variables with init button*/
        robot.init(hardwareMap);

        /*Vuforia Setup*/
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
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
        raiseArm(robot, 2360);

        extendLeadScrew(robot);

        //redBallKnock();

        //Start of vuforia
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        double vuforiaStart = getRuntime();

        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Time elapsed: ", getRuntime() - (vuforiaStart));
            telemetry.update();
            if(getRuntime() - vuforiaStart >= 2000) {
                vuMark = RelicRecoveryVuMark.CENTER;
            }
        }

        if(vuMark == RelicRecoveryVuMark.LEFT) {
            telemetry.addData("Driving to: ", vuMark + " column");
            telemetry.update();
            driveForward(straightSpeed, leftColumnTime);
            noDrive();
        }
        if(vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("Driving to: ", vuMark + " column");
            telemetry.update();
            driveForward(straightSpeed, middleColumnTime);
            noDrive();
        }
        if(vuMark == RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("Driving to: ", vuMark + " column");
            telemetry.update();
            driveForward(straightSpeed, rightColumnTime);
            noDrive();
        }

        sayAndPause("Turning: ", "Counter Clockwise", 500);
        turnClockwise(turnSpeed, turnTime, 250);

        sayAndPause("Driving: ", "Forward", breakTime);
        driveForward(0.2, 200);

        sayAndPause("Arm: ", "Lowering", breakTime);
        lowerArm(robot, 3360);

        sayAndPause("Claw: ", "Opening", 3 * breakTime);
        openingClaw(robot);

        sayAndPause("Arm: ", "Raising", breakTime);
        raiseArm(robot, 3360);

        sayAndPause("Claw ", "Closing", breakTime);
        driveForward(straightSpeed, 500);

        sayAndPause("Driving", "Back", breakTime);
        driveBackward(.2, 700);

        sayAndPause("Driving: ", "Forward", breakTime);
        driveForward(0.2, 1000);

        /* CODE FOR THE END OF THE PROGRAM*/

        /*Turns all motors off*/
        noDrive();
        robot.arm.setPower(0);
        robot.leadScrew.setPower(0);

        /*Declares end of program in telemetry*/
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();


    }
}
