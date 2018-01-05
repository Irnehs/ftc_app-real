/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Top Red Auto", group="Relic Recovery")

public class TopRedAutonomous extends RelicBaseAuto {


    /* Declare OpMode members. */

    //RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() throws InterruptedException {

        /*Shows that opMode loaded correctly*/
        sayAndPause("RelicAuto: ", "Connected", 0);

        /* Initialize the hardware variables with init button*/
        robot.init(hardwareMap);


        /*VARIABLE SETUP*/

        /*Driving*/
        long time2 = 500;
        double power2 = 0.5;
        long time1 = 1000;
        double power1 = 1;
        long boxTime = 500;
        long drivingPause = 500;
        double turnPower = 0.5;
        long turnTime = 500;


        /*Claw*/


        /*Telemetry Pause*/
        long telemetryPause = 250;


         /*Vuforia*/

        //Setup of camera monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //A license key is needed for Vuforia
        parameters.vuforiaLicenseKey = "AUAchNn/////AAAAGfqAcfY2+0TviBOpWNWvbFVO+Ki3ke54hx4bK3LAyMEOoMpSZ8pC6zWh9BQwmaUwpR8FxMbNylft5qxYuRVSaA5ijKZj2Gd5F4m8TKzk9YD+ZTRH0T/bzvhZLMr1IEnUKN0wyLqGqQqvI05qNqNahVd9OAHgy+MnrcWfrF1Ta1GUzQGc18K2qC7mioQFIJhc/KMCaFhmOer2sjtmxIp/kak0iDJfp77f/8kWvyV2IlnlR187HHWg1mgF9ZZspTYArFZa150FozF7PF7cR9xOuQZT7LuiwO/Ia64M/qa4vcOTlcHVtz6CVVC54KW1AAhQEg3p5kkG1hGbHJtvGovp7PKfragvZascLTnkCt4XK28C";

        //More camera setup
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Introduce pictographs to compare feed against
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        /*Setup Done*/

        // Send telemetry message to signify robot waiting after initialization
        sayAndPause("Status: ", "It is working and you loaded the package.", 0);

        /* Wait for the game to start (driver presses PLAY)*/
        waitForStart();

        /*GAME STARTS*/

        /*Activates Vuforia*/
        relicTrackables.activate();

        /*Declares start of OpMode in Telemetry*/
        sayAndPause("Status: ", "Auto Active", telemetryPause);

        /*Pick up block and extend lead screw*/

        //Close claw
        sayAndPause("Closing Claw", " ", telemetryPause);

        //Raise arm
        sayAndPause("Raising Arm", " ", telemetryPause);

        //Extend lead screw
        sayAndPause("Extending lead screw", " ", telemetryPause);

        /*Vuforia*/

        //Creates vuMark variable for detection
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //Starts clock in case image cannot be detected
        long clockStart = System.currentTimeMillis();

        //Scans for image until one is found
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && System.currentTimeMillis() - clockStart < 5000) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Detecting: ", vuMark);
            telemetry.update();
            sleep(50);
        }

        //Shows that image is detected in Telemetry
        telemetry.addData("Detecting: ", vuMark);
        telemetry.update();
        sleep(telemetryPause);

        /*Different first moves for different images*/

        //Goes one box short of middle if left is correct column
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            sayAndPause("Driving to: ", "Left", telemetryPause);
            driveBackward(power1, time1 + boxTime, drivingPause);
        }

        //Goes one box past middle if right is correct column
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            sayAndPause("Driving to: ", "Right", telemetryPause);
            driveBackward(power1, time1 - boxTime, drivingPause);
        }

        //Defaults to middle column and all positions are relative to it
        else {
            sayAndPause("Driving to: ", "Middle", telemetryPause);
            driveBackward(power1, time1, drivingPause);
        }

        //Adds stability by stopping and acts as a point for adding Telemetry
        sayAndPause("Driving to: ", "Placement position", telemetryPause);

        /*Turn*/
        sayAndPause("Turning: ", "90 clockwise", telemetryPause);
        turnCounterClockwise(turnPower, turnTime, drivingPause);

        /*Last drive train move*/

        //Moves robot forward into position to place glyphs
        driveForward(power2, time2, drivingPause);

        /*Place glyph into box*/

        //Lower arm to correct level
        sayAndPause("Lowering Arm", " ", telemetryPause);

        //Release claws
        sayAndPause("Releasing Claws", " ", telemetryPause);

        //Lift arm up
        sayAndPause("Lifting Arm Up", " ", telemetryPause);

        /*DONE*/

        /*Signify end with telemetry*/
        sayAndPause("Good Job, ", "All Done", telemetryPause);

        if (!opModeIsActive()) {

            /*Turns all motors off*/
            noDrive(0);
            robot.arm.setPower(0);
            robot.leadScrew.setPower(0);

            /*Declares end of program in telemetry*/
            telemetry.addData("Status: ", "Stopped");
            telemetry.update();
        }
    }
}