package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Created by kevinrockwell on 1/27/18.
 */

@Autonomous(name="Blue Auto", group = "Pushingbot")
public class BlueAuto extends RelicBaseAuto {

    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        //Check to make sure program is selected
        sayAndPause("RelicAuto: ", "Connected", 0);

        robot.init(hardwareMap); //Runs when init button is pressed, sets up robot

        sleep(500);

        robot.ballSwivel.setPosition(1);
        robot.ballLower.setPosition(0.1);

        // Send telemetry message to signify robot waiting after initialization
        sayAndPause("Status: ", "It is working and you loaded the package.", 0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status: ", "Auto Active"); //Checks to make sure Op Mode is running
        telemetry.update();

        //Starts at 1300(start + 1000) needed - 2240 +120 = 2360
        sayAndPause("Claw: ", "Closing", 500);
        closingClaw(robot);

        sayAndPause("Arm: ", "Raising", 500);
        raiseArm(robot, 2360);

        extendLeadScrew(robot);

        redBallKnock(robot);

        driveForward(0.5, 35, robot);

        telemetry.addData("Status: ", "Stopped"); //Says when program is over
        telemetry.update();

    }
}
