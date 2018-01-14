package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Feklaar on 12/9/2017.
 */

abstract class BaseOpMode extends LinearOpMode{

    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    public void MecanumDrive(double speed, double direction, double rotation, RelicRecoveryHardware robot) {
        final double v1 = speed * Math.cos(direction) + rotation;
        final double v2 = speed * Math.sin(direction) - rotation;
        final double v3 = speed * Math.sin(direction) + rotation;
        final double v4 = speed * Math.cos(direction) - rotation;

        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftBackMotor.setPower(v3);
        robot.rightBackMotor.setPower(v4);
    }

    public void openingClaw(RelicRecoveryHardware robot){
        robot.rightClaw.setPosition(0.9);
        robot.leftClaw.setPosition(0.0);
    }

    public void closingClaw(RelicRecoveryHardware robot) {
        robot.rightClaw.setPosition(.4);
        robot.leftClaw.setPosition(.6);
    }

    public void placeBlock(RelicRecoveryHardware robot) {
        robot.rightClaw.setPosition(.37);
        robot.leftClaw.setPosition(.63);
    }

}
