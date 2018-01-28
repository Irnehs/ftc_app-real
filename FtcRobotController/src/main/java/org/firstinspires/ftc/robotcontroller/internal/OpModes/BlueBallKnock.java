package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by kevinrockwell on 1/27/18.
 */
@Autonomous(name="BB Knock", group = "Pushingbot")
public class BlueBallKnock extends RelicBaseAuto {

    @Override
    public void runOpMode() throws InterruptedException {

        sayAndPause("RelicAuto: ","Connected",0);

        robot.init(hardwareMap); //Runs when init button is pressed, sets up robot

    sleep(500);

        robot.ballSwivel.setPosition(1);
        robot.ballLower.setPosition(0.1);

    // Send telemetry message to signify robot waiting after initialization
    sayAndPause("Status: ","It is working and you loaded the package.",0);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
        telemetry.addData("Status: ","Auto Active"); //Checks to make sure Op Mode is running
        telemetry.update();

    //Starts at 1300(start + 1000) needed - 2240 +120 = 2360
        blueBallKnock(robot);
    }
}
