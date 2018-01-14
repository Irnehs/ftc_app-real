package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Feklaar on 12/9/2017.
 */
/*CLASS FULL OF USEFUL METHODS THAT ARE COMMONLY REQUIRED*/
abstract class RelicBaseAuto extends BaseOpMode {
    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();
    String Version = "0.0.3";
    VuforiaLocalizer vuforia;

    /*Driving*/

    //Turning drive train off with time input
    public void noDrive(long time) {
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        sleep(time);
    }

    //Driving forward with power and time inputs
    public void driveForward(double power, long time, long pause) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
        noDrive(pause);
    }

    //Driving backward with power and time inputs
    public void driveBackward(double power, long time, long pause) {driveForward(-power, time, pause);}

    //Driving left with power and time inputs
    public void driveLeft(double power, long time, long pause) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(-power);
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
        noDrive(pause);
    }

    //Driving right with power and time inputs
    public void driveRight(double power, long time, long pause) {

        driveLeft(-power, time, pause);
    }

    //Turning clockwise with power and time inputs
    public void turnClockwise(double power, long time, long pause) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(-power);
        sleep(time);
        noDrive(pause);
    }

    //Turning counter clockwise with power and time inputs
    public void turnCounterClockwise(double power, long time, long pause) {
        turnClockwise(-power, time, pause);
    }

    public void sayAndPause(String title, String caption, long pause) {
        telemetry.addData(title, caption);
        telemetry.update();
        sleep(pause);
    }

    public void extendLeadScrew(RelicRecoveryHardware robot) {
        robot.leadScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Current Position", robot.leadScrew.getCurrentPosition());
        telemetry.update();
        robot.leadScrew.setTargetPosition((1120/2) * -30);
        robot.leadScrew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leadScrew.setPower(1);
    }

    public void raiseArm(RelicRecoveryHardware robot, int height){
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() + height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
    }

    public void lowerArm(RelicRecoveryHardware robot, int height) {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() - height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);

    }

}