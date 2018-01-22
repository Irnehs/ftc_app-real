package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public void noDrive() {
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
    }

    public void driveForward(double power, int inches) {
        int ticks = inches * (1120 / 4 * (int)Math.PI);
        telemetry.addData("Inches: ", inches);
        telemetry.addData("Ticks: ", ticks);
        telemetry.update();
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setTargetPosition(ticks);
        robot.rightBackMotor.setTargetPosition(ticks);
        robot.leftFrontMotor.setTargetPosition(ticks);
        robot.leftBackMotor.setTargetPosition(ticks);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Drivetrain: ", "Activating");
        telemetry.addData("Power: ", power);
        telemetry.update();
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        while(robot.rightFrontMotor.isBusy()) {/*Do nothing*/}
        telemetry.addData("Drivetrain: ", "Done");
        telemetry.update();
        noDrive();
    }

    /*Driving forward with power and time inputs
    public void driveForward(double power, long time) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
    }
    */

    //Driving backward with power and time inputs
    public void driveBackward(double power, int inches) {driveForward(power, -inches);}

    //Driving left with power and time inputs
   /* public void driveLeft(double power, long time, long pause) {
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
    public void turnClockwise(double power, int degrees) {
        robot.gyroSensor.calibrate();
        boolean notEqual;
        double wheelPower;


        if(degrees<0) {
            notEqual = robot.gyroSensor.getHeading() < degrees;
        }
        else {
            notEqual = robot.gyroSensor.getHeading() > degrees;
        }


        while(notEqual) {
            wheelPower = (degrees - robot.gyroSensor.getHeading())/(degrees/4);
            Range.clip(wheelPower,-1, 1);

            if(wheelPower < 0.1 && wheelPower > -0.1) {
                wheelPower = (wheelPower/Math.abs(wheelPower) * 0.1);
            }

            robot.rightFrontMotor.setPower(-wheelPower);
            robot.rightBackMotor.setPower(-wheelPower);
            robot.leftFrontMotor.setPower(wheelPower);
            robot.leftBackMotor.setPower(wheelPower);
        }

        noDrive();
    }*/

    public void turnCounterClockwise(double power, long time, long pause) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(-power);
        sleep(time);
        noDrive();
        sleep(pause);
    }

    //Turning counter clockwise with power and time inputs
    public void turnClockwise(double power, long time, long pause) {
        turnCounterClockwise(-power, time, pause);
    }

    public void sayAndPause(String title, String caption, long pause) {
        telemetry.addData(title, caption);
        telemetry.update();
        sleep(pause);
    }

    public void extendLeadScrew(RelicRecoveryHardware robot) {
        telemetry.addData("Lead Screw: ", "Moving");
        telemetry.update();
        robot.leadScrew.setPower(1);
        sleep(7000);
        robot.leadScrew.setPower(0);
        telemetry.addData("Lead Screw: ", "Stopped");
        telemetry.update();
    }

    public void raiseArm(RelicRecoveryHardware robot, int height){
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() + height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
    }

    public void blueBallKnock() {
        sayAndPause("Ball Knock Starting", "", 250);
        robot.ballLower.setPosition(0);
        sleep(250);
        if(robot.colorSensor.blue()+50<robot.colorSensor.red()) {
            sayAndPause("Ball Color: ", "Red", 250);
            robot.ballSwivel.setPower(-1);
            sleep(200);
            robot.ballSwivel.setPower(1);
            sleep(200);
        }
        else if(robot.colorSensor.red()+50<robot.colorSensor.blue()) {
            sayAndPause("Ball Color: ", "Blue", 250);
            robot.ballSwivel.setPower(1);
            sleep(200);
            robot.ballSwivel.setPower(-1);
            sleep(200);
        }
        sleep(250);
        robot.ballLower.setPosition(0.5);
    }

    public void redBallKnock() {
        sayAndPause("Ball Knock Starting", "", 250);
        robot.ballLower.setPosition(0);
        sleep(250);
        if(robot.colorSensor.blue()+50<robot.colorSensor.red()) {
            sayAndPause("Ball Color: ", "Red", 250);
            robot.ballSwivel.setPower(1);
            sleep(200);
            robot.ballSwivel.setPower(-1);
            sleep(200);
        }
        else if(robot.colorSensor.red()+50<robot.colorSensor.blue()) {
            sayAndPause("Ball Color: ", "Blue", 250);
            robot.ballSwivel.setPower(-1);
            sleep(200);
            robot.ballSwivel.setPower(1);
            sleep(200);
        }
        sleep(250);

        sleep(250);
        robot.ballLower.setPosition(0.5);
    }

    public void lowerArm(RelicRecoveryHardware robot, int height) {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() - height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);

    }

    public void jewelStart(RelicRecoveryHardware robot) {
        robot.ballLower.setPosition(0.8);
    }

}