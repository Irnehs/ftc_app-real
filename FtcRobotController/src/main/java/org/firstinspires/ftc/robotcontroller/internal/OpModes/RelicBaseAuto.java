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

    //Driving forward with power and time inputs
    public void driveForward(double power, long time) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
    }

    //Driving backward with power and time inputs
    public void driveBackward(double power, long time) {driveForward(-power, time);}

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

    public void blueBallKnock() {
        sayAndPause("Ball Knock Starting", "", 250);
        robot.ballLower.setPosition(0);
        sleep(250);
        if(robot.colorSensor.blue()+50<robot.colorSensor.red()) {
            sayAndPause("Ball Color: ", "Red", 250);
            robot.ballSwivel.setPosition(0.8);
        }
        else if(robot.colorSensor.red()+50<robot.colorSensor.blue()) {
            sayAndPause("Ball Color: ", "Blue", 250);
            robot.ballSwivel.setPosition(0.2);
        }
        sleep(250);
        robot.ballSwivel.setPosition(0.5);
        sleep(250);
        robot.ballLower.setPosition(0.5);
    }

    public void redBallKnock() {
        sayAndPause("Ball Knock Starting", "", 250);
        robot.ballLower.setPosition(0);
        sleep(250);
        if(robot.colorSensor.blue()+50<robot.colorSensor.red()) {
            sayAndPause("Ball Color: ", "Red", 250);
            robot.ballSwivel.setPosition(0.2);
        }
        if(robot.colorSensor.red()+50<robot.colorSensor.blue()) {
            sayAndPause("Ball Color: ", "Blue", 250);
            robot.ballSwivel.setPosition(0.8);
        }
        sleep(250);
        robot.ballSwivel.setPosition(0.5);
        sleep(250);
        robot.ballLower.setPosition(0.5);
    }

    public void lowerArm(RelicRecoveryHardware robot, int height) {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() - height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);

    }

}