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

@TeleOp(name="Auto", group="Pushingbot")
@Disabled
public class RelicAuto extends LinearOpMode {


    /* Declare OpMode members. */
    RelicRecoveryHardware naruto = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("RelicAuto: ", "Connected"); //Check to make sure program is selected
        telemetry.update();

        naruto.init(hardwareMap); //Runs when init button is pressed, sets up naruto

        double jewelMoverStart = naruto.jewelMover.getPosition();
        telemetry.addData("Jewel Mover Start: ", jewelMoverStart);
        telemetry.update();

        // Send telemetry message to signify naruto waiting;
        telemetry.addData("Status: ", "It is working and you loaded the package.");  //Check to make sure variable initialized correctly
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status: ", "Auto Active"); //Checks to make sure Op Mode is running
        telemetry.update();

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

        naruto.jewelMover.setPosition(0.5);
        sleep(1000);
        naruto.jewelMover.setPosition(0.0);
        telemetry.addData("Jewel Mover Start: ", jewelMoverStart);
        naruto.armSlide.setPower(1);
        sleep(1000);
        naruto.armSlide.setPower(0);



        telemetry.addData("Status: ", "Stopped"); //Says when program is over
    telemetry.update();

    }
}
