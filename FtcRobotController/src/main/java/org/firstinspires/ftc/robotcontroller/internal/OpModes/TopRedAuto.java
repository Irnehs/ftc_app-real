package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by kevinrockwell on 1/13/18.
 */

@Autonomous(name="Top Red Auto", group="Relic Recovery")

public class TopRedAuto extends RelicBaseAuto {

    String Version = "0.0.3";
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        /*Shows that opMode loaded correctly*/
        sayAndPause("RelicAuto: ", "Connected", 0);

        /* Initialize the hardware variables with init button*/
        robot.init(hardwareMap);

        waitForStart();

        //Starts at 1300(start + 1000) needed - 2240 +120 = 2360

        sayAndPause("Claw: ", "Closing", 500);
        closingClaw(robot);


        sayAndPause("Arm: ", "Raising", 500);
        raiseArm(robot, 2360);

        extendLeadScrew(robot);

        sayAndPause("Driving: ", "Forward", 500);
        driveForward(0.2, 1500, 1000);

        sayAndPause("Driving: ", "Left", 500);
        driveLeft(0.25, 1300, 1000);

        sayAndPause("Arm: ", "Lowering", 500);
        lowerArm(robot, 3360);

        sayAndPause("Claw: ", "Opening", 500);
        placeBlock(robot);

        sayAndPause("Arm: ", "Raising", 500);
        raiseArm(robot, 3360);

        sayAndPause("Claw ", "Closing", 500);
        driveForward(0.2, 500, 250);

        sayAndPause("Driving", "Back", 500);
        driveBackward(.2, 700, 500);

        sayAndPause("Driving: ", "Forward", 500);
        driveForward(0.2, 800, 1000);

        /* CODE FOR THE END OF THE PROGRAM*/

        /*Turns all motors off*/
        noDrive(100);
        robot.arm.setPower(0);
        robot.leadScrew.setPower(0);

        /*Declares end of program in telemetry*/
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();


    }
}
