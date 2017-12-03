package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestEncoders", group="TestEncoder")
public class TestEncoders extends LinearOpMode {

    RelicTestingHardware Marvin = new RelicTestingHardware();
    String Version = "0.0.3";

    @Override
    public void runOpMode() throws InterruptedException {


        Marvin.init(hardwareMap);


        waitForStart();
    }
}
