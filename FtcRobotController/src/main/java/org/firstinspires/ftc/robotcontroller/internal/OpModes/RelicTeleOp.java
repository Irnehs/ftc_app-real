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

@TeleOp(name="TeleOp", group="TeleOp")
@Disabled
public class RelicTeleOp extends LinearOpMode {


    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    // could also use HardwarePushbotMatrix class.

    /*Create the method for setting the wheels' power*/

    //Variables for the wheels power
    double rightFrontPower;
    double rightBackPower;
    double leftFrontPower;
    double leftBackPower;

    //Actual method
    public void wheelPower(double frontRight, double frontLeft, double backRight, double backLeft) {
        rightFrontPower = frontRight;
        leftFrontPower = frontLeft;
        rightBackPower = backRight;
        leftBackPower = backLeft;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("RelicTeleOp: ", "Connected"); //Check to make sure program is selected
        telemetry.update();

        /*INITIALIZATION OF ALL VARIABLES*/

        /*Gamepad 1*/

        //Dpad control of drivetrain
        boolean forward;
        boolean backward;
        boolean leftward;
        boolean rightward;
        boolean aButtonGp1;
        boolean bButtonGp1;
        double adjustmentSpeed = 0.5;

        //Bumpers for Gamepad 1
        boolean leftBumperGp1;
        boolean rightBumperGp1;

        /*Gamepad 2*/

        //Glyph arm
        double armLevel = 1;
        boolean armUp;
        boolean armDown;
        int armRevolutions = 1/4;
        boolean aButtonGp2;
        boolean bButtonGp2;
        boolean xButtonGp1;




        robot.init(hardwareMap); //Runs when init button is pressed, sets up robot

        //double jewelMoverStart = robot.jewelArm.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "It is working and you loaded the package.");  //Check to make sure variable initialized correctly
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status: ", "TeleOp Active"); //Checks to make sure Op Mode is running
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //Runs as long as stop is not on screen and play triangle has been clicked

            /*GET CONTROLLER VALUES*/

            /*Gamepad 1*/

            forward = gamepad1.dpad_up;     //Up dpad
            backward = gamepad1.dpad_down;  //Down dpad
            leftward = gamepad1.dpad_left;  //Left dpad
            rightward = gamepad1.dpad_right;//Right dpad
            aButtonGp1 = gamepad1.a;        //A
            bButtonGp1 = gamepad1.b;        //B
            rightBumperGp1 = gamepad1.right_bumper; //Right bumper
            leftBumperGp1 = gamepad1.left_bumper;   //Left bumper

            /*Gamepad 2*/

            armUp = gamepad2.right_bumper;  //Right bumper
            armDown = gamepad2.left_bumper; //Left bumper
            aButtonGp2 = gamepad2.a;        //A
            bButtonGp2 = gamepad2.b;        //B
            xButtonGp1 = gamepad1.x;        //X


            /*Joystick Control*/

            //Converts joystick output into variables used to calculate power for the wheels
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            //Uses created variables to calculate power for the wheels
            leftFrontPower = r * Math.cos(robotAngle) + rightX;
            rightFrontPower = r * Math.sin(robotAngle) - rightX;
            leftBackPower = r * Math.sin(robotAngle) + rightX;
            rightBackPower = r * Math.cos(robotAngle) - rightX;

            //Limits the wheel power to acceptable inputs
            Range.clip(leftFrontPower,-1, 1);
            Range.clip(rightFrontPower, -1, 1);
            Range.clip(leftBackPower, -1, 1);
            Range.clip(rightBackPower, -1, 1);

            /*Dpad control: WARNING: OVERRIDES JOYSTICK CONTROL*/

            //Adjusting the speed for dpad control
            if(aButtonGp1) {adjustmentSpeed+=0.1;} //A on gamepad 1 increases adjustment speed
            if(bButtonGp1) {adjustmentSpeed-=0.1;} //B on gamepad 2 decreases adjustment speed
            telemetry.addData("Adjustment Speed: ", adjustmentSpeed);
            telemetry.update();

            /*If dpad up pressed, drive forwards at adjustment speed*/
            while(forward) {
                wheelPower(adjustmentSpeed, adjustmentSpeed, adjustmentSpeed, adjustmentSpeed); //Sets new wheel power
            }
            while(backward) {//If dpad down pressed, drive backwards at adjustment speed
                wheelPower(-adjustmentSpeed, -adjustmentSpeed, -adjustmentSpeed, -adjustmentSpeed);// Sets new wheel power
            }

            while(leftward) {            //If dpad left pressed, drive left at adjustment speed
                wheelPower(adjustmentSpeed, -adjustmentSpeed, -adjustmentSpeed, adjustmentSpeed); //Sets new wheel power
            }

            while(rightward) {          //If dpad right pressed, drive right at adjustment speed
                wheelPower(-adjustmentSpeed, adjustmentSpeed, adjustmentSpeed, -adjustmentSpeed); //Sets new wheel Power
            }

            //TURNS WHEELS ON
            robot.leftFrontMotor.setPower(leftFrontPower);
            robot.rightFrontMotor.setPower(rightFrontPower);
            robot.leftBackMotor.setPower(leftBackPower);
            robot.rightBackMotor.setPower(rightBackPower);
/*
            //Test Jewel Arm
            if(rightBumperGp1){
                robot.jewelMover.setPosition(0.5);
            } //Go to 90 degrees
            if(leftBumperGp1) {
                robot.jewelMover.setPosition(jewelMoverStart);
            } //Return to start
*/
            /*Glyph Arm*/

            //Sets the increasing levels
            if(armUp && armLevel <= 4) {
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setTargetPosition(1680*armRevolutions);
                armLevel++;
            }

            //Sets the decreasing levels
            if(armDown && armLevel >= 1) {
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setTargetPosition(1680*-armRevolutions);
                armLevel--;
            }

            /*Claw*/

            //Closes claws
            if(aButtonGp2) {
                robot.leftClaw.setPosition(0.25);
                robot.rightClaw.setPosition(0.25);
            }

            //Opens claws
            if(bButtonGp2) {
                robot.leftClaw.setPosition(0.8);
                robot.rightClaw.setPosition(0.8);
            }


            telemetry.addData("Arm Level: ", armLevel);
            telemetry.update();
            telemetry.addData("Status: ", "Updated"); //Checks to make sure it ran correctly
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

        }

        /*Code for the end of the program*/
        if(!opModeIsActive()){
            telemetry.addData("Status: ", "Stopped"); //Says when program is over
            telemetry.update();
        }
    }
}
