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
//1.5
//1
//1.5


@Autonomous(name="Top Blue Auto", group="Relic Recovery")

public class TopBlueAuto extends RelicBaseAuto {


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

        waitForStart();

        //Starts at 1300(start + 1000) needed - 2240 +120 = 2360

        sayAndPause("Claw: ", "Closing", 500);
        closingClaw(robot);


        sayAndPause("Arm: ", "Raising", 500);
        raiseArm(robot, 2360);

        extendLeadScrew(robot);

        sayAndPause("Driving: ", "Forward", 500);
        driveForward(0.2, 1500, 1000);

        sayAndPause("Driving: ", "Right", 500);
        driveRight(0.25, 1300, 1000);

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