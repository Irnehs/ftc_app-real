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

        /*Drive variables*/
        long leftColumnTime = 1000;
        long middleColumnTime = 1200;
        long rightColumnTime = 1400;
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

        sayAndPause("Driving: ", "Forward", breakTime);
        driveForward(straightSpeed, 500);

        sayAndPause("Turning: ", "Clockwise", breakTime);
        turnClockwise(turnSpeed, turnTime,500 );

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
        turnCounterClockwise(turnSpeed, turnTime, 250);

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