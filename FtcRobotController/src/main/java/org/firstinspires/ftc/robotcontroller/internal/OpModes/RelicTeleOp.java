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

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("RelicTeleOp: ", "Connected"); //Check to make sure program is selected
        telemetry.update();


        boolean forward; //Used for dpad control
        boolean backward; //^
        boolean leftward; //^
        boolean rightward;//^
        boolean aButtonGp1;//^
        boolean bButtonGp1;//^
        boolean leftBumperGp1;
        boolean rightBumperGp1;
        double adjustmentSpeed = 0.5; //^
        double armLevel = 1;
        boolean armUp;
        boolean armDown;
        int armRevolutions = 1/4;
        boolean aButtonGp2;
        boolean bButtonGp2;

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
            //Get controller values for gamepads
            //GAMEPAD 1
            forward = gamepad1.dpad_up;     //Up dpad
            backward = gamepad1.dpad_down;  //Down dpad
            leftward = gamepad1.dpad_left;  //Left dpad
            rightward = gamepad1.dpad_right;//Right dpad
            aButtonGp1 = gamepad1.a;        //A
            bButtonGp1 = gamepad1.b;        //B
            rightBumperGp1 = gamepad1.right_bumper; //Right bumper
            leftBumperGp1 = gamepad1.left_bumper;   //Left bumper
            armUp = gamepad2.right_bumper;
            armDown = gamepad2.left_bumper;
            aButtonGp2 = gamepad2.a;
            bButtonGp2 = gamepad2.b;


            //Joystick control
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

            if(leftward) {            //If dpad left pressed, drive left at adjustment speed
                leftFrontPower = adjustmentSpeed; //Sets new motor power
                rightFrontPower = -adjustmentSpeed; //^
                leftBackPower = -adjustmentSpeed;  //^
                rightBackPower = adjustmentSpeed;  //^
            }

            if(rightward) {          //If dpad right pressed, drive right at adjustment speed
                leftFrontPower = -adjustmentSpeed;  //Sets new motor power
                rightFrontPower = adjustmentSpeed; //^
                leftBackPower = adjustmentSpeed;   //^
                rightBackPower = -adjustmentSpeed; //^
            }

            robot.leftFrontMotor.setPower(leftFrontPower);  //Sets power
            robot.rightFrontMotor.setPower(rightFrontPower); //^
            robot.leftBackMotor.setPower(leftBackPower);    //^
            robot.rightBackMotor.setPower(rightBackPower);  //^
/*
            //Test Jewel Arm
            if(rightBumperGp1){
                robot.jewelMover.setPosition(0.5);
            } //Go to 90 degrees
            if(leftBumperGp1) {
                robot.jewelMover.setPosition(jewelMoverStart);
            } //Return to start
*/
            //Control the arm

            if(armUp && armLevel <= 4) {
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setTargetPosition(1680*armRevolutions);
                armLevel++;
            }

            if(armDown && armLevel >= 1) {
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setTargetPosition(1680*-armRevolutions);
                armLevel--;
            }

            if(aButtonGp2) {
                robot.leftClaw.setPosition(0.25);
                robot.rightClaw.setPosition(0.25);
            }

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
        if(!opModeIsActive()){
            telemetry.addData("Status: ", "Stopped"); //Says when program is over
            telemetry.update();
        }
    }
}
