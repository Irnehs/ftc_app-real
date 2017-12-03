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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the naruto.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the naruto FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeravoltzRemoteOpMode", group="MainOpModes")
public class TeravoltzRemoteOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    RelicTestingHardware naruto = new RelicTestingHardware();
    String Version = "0.0.3";


    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        naruto.init(hardwareMap);

        //name of buttons
        boolean armUp;
        boolean armDown;
        boolean clawOpen;
        boolean clawClose;
        boolean leadScrewOut;
        boolean leadScrewIn;
        int rotation = 1120;
        int halfRotation = rotation / 2;
        int quarterRotation = halfRotation / 2;
        boolean forward; //Used for dpad control
        boolean backward; //^
        boolean left; //^
        boolean right;//^
        boolean aButtonGp1;
        boolean bButtonGp1;
        double adjustmentSpeed = 0.5; //^



        // Send telemetry message to signify naruto waiting
        //say("Ready", "It is working and you loaded the package.");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {

            armUp = gamepad2.a;
            armDown = gamepad2.b;
            clawOpen = gamepad2.right_bumper;
            clawClose = gamepad2.left_bumper;
            leadScrewIn = gamepad1.right_bumper;
            leadScrewOut = gamepad1.left_bumper;
            forward = gamepad1.dpad_up;     //Up dpad
            backward = gamepad1.dpad_down;  //Down dpad
            left = gamepad1.dpad_left;  //Left dpad
            right = gamepad1.dpad_right;//Right dpad
            aButtonGp1 = gamepad1.a;        //A
            bButtonGp1 = gamepad1.b;        //B
            int currentPos = naruto.arm.getCurrentPosition();

            if (armUp) {
                telemetry.addData("Current Position", naruto.arm.getCurrentPosition());
                telemetry.update();

                naruto.arm.setTargetPosition(currentPos + halfRotation - 30);

                telemetry.addData("Target:", naruto.arm.getTargetPosition());
                telemetry.update();
                sleep(500);

                naruto.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (armDown) {
                telemetry.addData("Current Position", naruto.arm.getCurrentPosition());
                telemetry.update();

                naruto.arm.setTargetPosition(currentPos - halfRotation - 50);

                telemetry.addData("Target:", naruto.arm.getTargetPosition());
                telemetry.update();
                sleep(500);

                naruto.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                naruto.arm.setPower(0.45);
            }

            if (armUp || armDown) {
                naruto.arm.setPower(0.45);
                armDown = armUp = false;
            }

            if (!(naruto.arm.isBusy())) {
                naruto.arm.setPower(0);
                naruto.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (clawOpen) {
                naruto.rightClaw.setPosition(.30);
                naruto.leftClaw.setPosition(.75);
            }

            else if (clawClose) {
                naruto.rightClaw.setPosition(.875);
                naruto.leftClaw.setPosition(0);
            }

            if (leadScrewIn){
                telemetry.addData("Current Position", naruto.leadScrew.getCurrentPosition());
                telemetry.update();

                naruto.leadScrew.setTargetPosition(currentPos + rotation * -16);

                telemetry.addData("Target:", naruto.leadScrew.getTargetPosition());
                telemetry.update();
                sleep(500);

                naruto.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else if (leadScrewOut){
                telemetry.addData("Current Position", naruto.leadScrew.getCurrentPosition());
                telemetry.update();

                naruto.leadScrew.setTargetPosition(currentPos + rotation * 16);

                telemetry.addData("Target:", naruto.leadScrew.getTargetPosition());
                telemetry.update();
                sleep(500);

                naruto.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (leadScrewIn || leadScrewOut) {
                naruto.leadScrew.setPower(.45);
                leadScrewIn = leadScrewOut = false;
            }

            if (!(naruto.leadScrew.isBusy())) {
                naruto.leadScrew.setPower(0);
            }

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);                        //Converts joystick to usable data,
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4; //^
            double rightX = gamepad1.right_stick_x;                                                     //^

            double leftFrontPower = r * Math.cos(robotAngle) + rightX; //Calculates power
            double rightFrontPower = r * Math.sin(robotAngle) - rightX; //^
            double leftBackPower = r * Math.sin(robotAngle) + rightX;   //^
            double rightBackPower = r * Math.cos(robotAngle) - rightX; //^

            Range.clip(leftFrontPower,-1, 1);  //Limits values to acceptable motor inputs
            Range.clip(rightFrontPower, -1, 1); //^
            Range.clip(leftBackPower, -1, 1);   //^
            Range.clip(rightBackPower, -1, 1); //^

            //WARNING: DPAD CONTROL OVERRIDES JOYSTICK CONTROL

            //Dpad control
            if(aButtonGp1) {adjustmentSpeed++;} //A on gamepad 1 increases adjustment speed
            if(bButtonGp1) {adjustmentSpeed--;} //B on gamepad 2 decreases adjustment speed

            if(forward) {              //If dpad up pressed, drive forwards at adjustment speed
                leftFrontPower = -adjustmentSpeed;  //Sets new motor power
                rightFrontPower = -adjustmentSpeed;  //^
                leftBackPower = -adjustmentSpeed;    //^
                rightBackPower = -adjustmentSpeed;  //^
            }

            if(backward) {             //If dpad down pressed, drive backwards at adjustment speed
                leftFrontPower = adjustmentSpeed; //Sets new motor power
                rightFrontPower = adjustmentSpeed; //^
                leftBackPower = adjustmentSpeed;   //^
                rightBackPower = adjustmentSpeed; //^
            }

            if(left) {            //If dpad left pressed, drive left at adjustment speed
                leftFrontPower = adjustmentSpeed; //Sets new motor power
                rightFrontPower = -adjustmentSpeed; //^
                leftBackPower = -adjustmentSpeed;  //^
                rightBackPower = adjustmentSpeed;  //^
            }

            if(right) {          //If dpad right pressed, drive right at adjustment speed
                leftFrontPower = -adjustmentSpeed;  //Sets new motor power
                rightFrontPower = adjustmentSpeed; //^
                leftBackPower = adjustmentSpeed;   //^
                rightBackPower = -adjustmentSpeed; //^
            }

            naruto.leftFrontMotor.setPower(leftFrontPower);  //Sets power
            naruto.rightFrontMotor.setPower(rightFrontPower); //^
            naruto.leftBackMotor.setPower(leftBackPower);    //^
            naruto.rightBackMotor.setPower(rightBackPower);  //^
        }



            if (!opModeIsActive()) {
                telemetry.addData("Status: ", "Stopped"); //Says when program is over
                telemetry.update();
            }
        }
    }
