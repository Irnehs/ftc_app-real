package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Feklaar on 12/9/2017.
 */
/*CLASS FULL OF USEFUL METHODS THAT ARE COMMONLY REQUIRED*/
abstract class RelicBaseAuto extends LinearOpMode {
    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();
    String Version = "0.0.3";
    VuforiaLocalizer vuforia;

    /*Driving*/

    //Turning drive train off with time input
    public void noDrive(long time) {
        driveForward(0, 0, time);
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
}