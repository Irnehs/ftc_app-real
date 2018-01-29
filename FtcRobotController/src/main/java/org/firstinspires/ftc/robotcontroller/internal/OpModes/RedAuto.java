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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name="Red Auto", group="Pushingbot")
public class RedAuto extends RelicBaseAuto {


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

        blueBallKnock(robot);

        driveBackward(0.5, 35, robot);

        telemetry.addData("Status: ", "Stopped"); //Says when program is over
        telemetry.update();

    }
}



/* Steps of autonomous

        Pick up block
        Knocking off the ball (30 pts)
            * Sense the ball's color
            * Drive forwards or backwards
        Drive to a place within view of the image
        Use the image to identify correct column
        Drive to the correct column
        Insert block into correct column (45 pts)
        Park on base in front of crytoboxes (10 pts)
         */