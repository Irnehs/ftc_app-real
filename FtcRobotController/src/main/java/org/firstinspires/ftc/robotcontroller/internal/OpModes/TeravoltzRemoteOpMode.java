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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RemoteOpMode", group="TeleOp")
public class TeravoltzRemoteOpMode extends LinearOpMode {



    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();
    String Version = "0.0.3";


    // could also use HardwarePushbotMatrix class.

    /*CREATES THE METHOD FOR SETTING THE WHEEL'S POWER*/

    /*Variables for the wheels power*/
    double rightFrontPower;
    double rightBackPower;
    double leftFrontPower;
    double leftBackPower;

    /*Actual method*/
    public void wheelPower(double frontRight, double frontLeft, double backRight, double backLeft) {
        rightFrontPower = frontRight;
        leftFrontPower = frontLeft;
        rightBackPower = backRight;
        leftBackPower = backLeft;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables*/
        robot.init(hardwareMap);

        /*INITIALIZATION OF ALL VARIABLES*/

        /*Glyph Arm Variables*/

        int rotation = 1120;
        int halfRotation = rotation / 2;
        int quarterRotation = halfRotation / 2;
        double armPosition = 1;

        /*Drivetrain Variables*/

        //Dpad
        double unidirectionalSpeed = 0;

        /*Gamepad 1*/

        boolean forward = gamepad1.dpad_up;     //Up dpad
        boolean backward = gamepad1.dpad_down;  //Down dpad
        boolean left = gamepad1.dpad_left;      //Left dpad
        boolean right = gamepad1.dpad_right;    //Right dpad
        boolean aButtonGp1 = gamepad1.a;        //A
        boolean bButtonGp1 = gamepad1.b;        //B
        boolean leadScrewIn = gamepad1.right_bumper; //Right bumper
        boolean leadScrewOut = gamepad1.left_bumper; //Left bumper

        /*Gamepad 2*/

        boolean armUp = gamepad2.a;                    //A
        boolean armDown = gamepad2.b;                  //B
        boolean clawOpen = gamepad2.right_bumper;      //Right bumper
        boolean clawClose = gamepad2.left_bumper;      //Left bumper

        /*END OF SETUP*/

        /* Send telemetry message to signify robot waiting*/
        telemetry.addData("Ready: ", "It is working and you loaded the package.");
        telemetry.update();

        /* Wait for the game to start, which is when the driver presses PLAY*/
        waitForStart();

        /*START OF LOOP THAT RUNS REPEATEDLY*/
        while (opModeIsActive()) {

            /*GET CONTROLLER AND ROBOT VALUES*/

            /*Gamepad 1*/

            forward = gamepad1.dpad_up;     //Up dpad
            backward = gamepad1.dpad_down;  //Down dpad
            left = gamepad1.dpad_left;      //Left dpad
            right = gamepad1.dpad_right;    //Right dpad
            aButtonGp1 = gamepad1.a;        //A
            bButtonGp1 = gamepad1.b;        //B
            leadScrewIn = gamepad1.right_bumper; //Right bumper
            leadScrewOut = gamepad1.left_bumper; //Left bumper

            /*Gamepad 2*/

            armUp = gamepad2.a;                    //A
            armDown = gamepad2.b;                  //B
            clawOpen = gamepad2.right_bumper;      //Right bumper
            clawClose = gamepad2.left_bumper;      //Left bumper

            /*Robot*/

            int currentPos = robot.arm.getCurrentPosition(); // Stores current arm position


            /*GLYPH ARM CONTROL*/

            /*Sets position for raising of the glyph arm*/
            if (armUp && armPosition < 5) {
                telemetry.addData("Current Position", robot.arm.getCurrentPosition());
                telemetry.update();

                robot.arm.setTargetPosition(currentPos + halfRotation - 30);

                telemetry.addData("Target:", robot.arm.getTargetPosition());
                telemetry.update();
                sleep(500);

                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPosition = armPosition + 1;
            }

            /*Sets position for lowering of the glyph arm*/
            if (armDown && armPosition > 1) {
                telemetry.addData("Current Position", robot.arm.getCurrentPosition());
                telemetry.update();

                robot.arm.setTargetPosition(currentPos - halfRotation - 50);

                telemetry.addData("Target:", robot.arm.getTargetPosition());
                telemetry.update();
                sleep(500);

                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPosition = armPosition - 1;
            }

            /*Sets arm power and actually makes it run*/
            if (armUp || armDown) {
                robot.arm.setPower(0.45);
                armDown = armUp = false;
            }

            /*Turns glyph arm off after it reaches target*/
            if (!(robot.arm.isBusy())) {
                robot.arm.setPower(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            /*CLAW CONTROL*/

            /*Opens the claw*/
            if (clawOpen) {
                robot.rightClaw.setPosition(.30);
                robot.leftClaw.setPosition(.75);
            }

            /*Closes the claw*/
            else if (clawClose) {
                robot.rightClaw.setPosition(.875);
                robot.leftClaw.setPosition(0);
            }

            /*LEAD SCREW CONTROL*/

            /*Sets position for retraction of lead screw*/
            if (leadScrewIn) {
                telemetry.addData("Current Position", robot.leadScrew.getCurrentPosition());
                telemetry.update();

                robot.leadScrew.setTargetPosition(currentPos + rotation * -16);

                telemetry.addData("Target:", robot.leadScrew.getTargetPosition());
                telemetry.update();

                robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            /*Sets position for extension of lead screw*/
            else if (leadScrewOut) {
                telemetry.addData("Current Position", robot.leadScrew.getCurrentPosition());
                telemetry.update();

                robot.leadScrew.setTargetPosition(currentPos + rotation * 16);

                telemetry.addData("Target:", robot.leadScrew.getTargetPosition());
                telemetry.update();

                robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            /*Sets lead screw power and actually makes it run*/
            if (leadScrewIn || leadScrewOut) {
                robot.leadScrew.setPower(.45);
                leadScrewIn = leadScrewOut = false;
            }

            /*Turns lead screw off when target reached*/
            if (!(robot.leadScrew.isBusy())) {
                robot.leadScrew.setPower(0);
            }


            /*JOYSTICK CONTROL*/

            /*Converts joystick output into variables used to calculate power for the wheels*/
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            /*Uses created variables to calculate power for the wheels*/
            leftFrontPower = r * Math.cos(robotAngle) - rightX;
            rightFrontPower = r * Math.sin(robotAngle) + rightX;
            leftBackPower = r * Math.sin(robotAngle) - rightX;
            rightBackPower = r * Math.cos(robotAngle) + rightX;

            /*Limits values to acceptable motor inputs*/
            Range.clip(leftFrontPower,-1, 1);
            Range.clip(rightFrontPower, -1, 1);
            Range.clip(leftBackPower, -1, 1);
            Range.clip(rightBackPower, -1, 1);

            /*Adds aall values to*/
            telemetry.addData("Left Front Power: ", leftFrontPower);
            telemetry.addData("Right Front Power: ", rightFrontPower);
            telemetry.addData("Left Back Power: ", leftBackPower);
            telemetry.addData("Right Back Power: ", rightBackPower);

            /*DPAD CONTROL: WARNING: OVERRIDES JOYSTICK CONTROL*/

            /*Adjusting the speed for dpad control*/
            if(aButtonGp1) {unidirectionalSpeed+=0.05;} //A on gamepad 1 increases adjustment speed
            if(bButtonGp1) {unidirectionalSpeed-=0.05;} //B on gamepad 1 decreases adjustment speed
            telemetry.addData("Adjustment Speed: ", unidirectionalSpeed); //displays current adjustment speed
            telemetry.update();

            /*While dpad up pressed, drive forwards at adjustment speed*/
            while(forward) {
                wheelPower(unidirectionalSpeed, unidirectionalSpeed, unidirectionalSpeed, unidirectionalSpeed);
            }

            /*If dpad down pressed, drive backwards at adjustment speed*/
            while(backward) {
                wheelPower(-unidirectionalSpeed, -unidirectionalSpeed, -unidirectionalSpeed, -unidirectionalSpeed);
            }

            /*While dpad left pressed, drive left at adjustment speed*/
            while(left) {            //If dpad left pressed, drive left at adjustment speed
                wheelPower(unidirectionalSpeed, -unidirectionalSpeed, -unidirectionalSpeed, unidirectionalSpeed);
            }

            /*While dpad right pressed, drive right at adjustment speed*/
            while(right) {
                wheelPower(-unidirectionalSpeed, unidirectionalSpeed, unidirectionalSpeed, -unidirectionalSpeed);
            }

            /*TURNS WHEELS ON*/
            robot.leftFrontMotor.setPower(leftFrontPower);
            robot.rightFrontMotor.setPower(rightFrontPower);
            robot.leftBackMotor.setPower(leftBackPower);
            robot.rightBackMotor.setPower(rightBackPower);

        }

        /* CODE FOR THE END OF THE PROGRAM*/

            if (!opModeIsActive()) {

                /*Turns all motors off*/
                wheelPower(0, 0, 0, 0);
                robot.arm.setPower(0);
                robot.leadScrew.setPower(0);

                /*Declares end of program in telemetry*/
                telemetry.addData("Status: ", "Stopped");
                telemetry.update();
            }

        }
    }
