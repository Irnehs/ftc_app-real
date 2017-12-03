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


@TeleOp(name="TestArm", group="Test")
public class TestArm extends LinearOpMode {


    /* Declare OpMode members. */
    RelicTestingHardware naruto = new RelicTestingHardware();
    String Version = "0.0.3";

    public int ticksPerRev = 1120;
    public double downGear = 4;


    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {


        naruto.init(hardwareMap);







        waitForStart();


        //Encoders

        /*
        naruto.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Current Position", naruto.arm.getCurrentPosition());
        telemetry.update();

        naruto.arm.setTargetPosition(ticksPerRev);

        telemetry.addData("Target:", naruto.arm.getTargetPosition());
        telemetry.update();
        sleep(500);

        naruto.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        naruto.arm.setPower(0.2);

        while(naruto.arm.isBusy()) {
        //Motor is moving
            telemetry.addData("position", naruto.arm.getCurrentPosition());
            telemetry.update();
        }

        naruto.arm.setPower(0);
        telemetry.addData("Position", naruto.arm.getCurrentPosition());
        telemetry.update();
        naruto.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //positive rotates motor clockwise and arm up
        //Opposite of lead screw
        // Test code for arm using power (power already reversed)

        */
        //Manual reverse
        //naruto.leadScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //naruto.leadScrew.setPower(0.2);
        //sleep(10000);


        naruto.arm.setPower(0.3);
        sleep(500);
        naruto.arm.setPower(0);
        sleep(1000);
        naruto.arm.setPower(-0.3);
        sleep(500);
        naruto.waitForTick(40);

    }
}
