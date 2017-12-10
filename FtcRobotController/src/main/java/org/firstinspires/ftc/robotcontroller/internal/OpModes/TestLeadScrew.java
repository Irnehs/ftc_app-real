package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestLeadScrew", group="TestLeadScrew")
public class TestLeadScrew extends LinearOpMode {


    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();
    String Version = "0.0.3";

    //104/60=1.733333
    //1.733333*.5=.866666
    //.8666666*4=3.466666


    public int singleRotation = 1120;
    public int halfRotation = singleRotation / 2;


    // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);


        waitForStart();
        //Negative moves lead screw forwards
        //104 RPM at one power


        robot.leadScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Current Position", robot.leadScrew.getCurrentPosition());
        telemetry.update();

        robot.leadScrew.setTargetPosition(singleRotation * -16);

        telemetry.addData("Target:", robot.leadScrew.getTargetPosition());
        telemetry.update();
        sleep(500);

        robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leadScrew.setPower(1);

        while(robot.leadScrew.isBusy()) {
            //Motor is moving
            telemetry.addData("position", robot.leadScrew.getCurrentPosition());
            telemetry.update();
        }

        robot.leadScrew.setPower(0);
        telemetry.addData("Position", robot.leadScrew.getCurrentPosition());
        telemetry.update();
        robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leadScrew.setTargetPosition(singleRotation * 0);

        telemetry.addData("Target:", robot.leadScrew.getTargetPosition());
        telemetry.update();
        sleep(500);

        robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leadScrew.setPower(1);

        while(robot.leadScrew.isBusy()) {
            //Motor is moving
            telemetry.addData("position", robot.leadScrew.getCurrentPosition());
            telemetry.update();
        }

        robot.leadScrew.setPower(0);
        telemetry.addData("Position", robot.leadScrew.getCurrentPosition());
        telemetry.update();
        robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        /*robot.leadScrew.setPower(0.5);
        sleep(4000);
        robot.leadScrew.setPower(0);
        sleep(500);
        robot.leadScrew.setPower(-0.5);
        sleep(4000);
        robot.leadScrew.setPower(0);
        sleep(100);
        */
    }
}