package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    //Driving

    int centerDistance = 39;

    //Turning drive train off with time input
    public void noDrive(RelicRecoveryHardware robot) {
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
    }

    public void driveForward(double power, int inches, RelicRecoveryHardware robot) {
        double _ticks = inches * (1120 / (4 * Math.PI));
        int ticks = (int)_ticks;
        telemetry.addData("Inches: ", inches);
        telemetry.addData("Ticks: ", ticks);
        telemetry.update();

        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        while(robot.leftFrontMotor.isBusy()) {
            telemetry.addData("rightFront position: ", robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("leftFront position: ", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("rightBack position: ", robot.rightBackMotor.getCurrentPosition());
            telemetry.addData("leftBack position: ", robot.leftBackMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drivetrain: ", "Done");
        telemetry.update();
        noDrive(robot);
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
    public void driveBackward(double power, int inches, RelicRecoveryHardware robot) {
        driveForward(power, -inches, robot);
    }


    //Distance between wheels is  = 14.525
    public void turnCounterClockwise(double power, int degrees, RelicRecoveryHardware robot) {
        //Setup. Declare ticks & ticks to degrees
        /* ticksPerDegree is circumference of the circle determined by the distances between the
           adjacent wheels on the robot, turned to motor ticks, then divided by 60 */
        double ticksPerDegree = 20.5;
        double _ticks = degrees * ticksPerDegree; //precision
        int ticks = (int)_ticks;

        telemetry.addData("Degrees: ", degrees);
        telemetry.addData("Ticks: ", ticks);
        telemetry.update();

        //reset motor encoders to be safe
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set motor ticks and run to target
        robot.rightFrontMotor.setTargetPosition(ticks);
        robot.rightBackMotor.setTargetPosition(ticks);
        robot.leftFrontMotor.setTargetPosition(-ticks);
        robot.leftBackMotor.setTargetPosition(-ticks);

        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Drivetrain: ", "Activating");
        telemetry.addData("Power: ", power);
        telemetry.update();

        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(-power);

        //stop at end of turn
        while(robot.leftFrontMotor.isBusy()) {
            telemetry.addData("rightFront position: ", robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("leftFront position: ", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("rightBack position: ", robot.rightBackMotor.getCurrentPosition());
            telemetry.addData("leftBack position: ", robot.leftBackMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drivetrain: ", "Done");
        telemetry.update();
        noDrive(robot);
    }

    //Turning counter clockwise with power and time inputs
    public void turnClockwise(double power, int degrees, long pause, RelicRecoveryHardware robot) {
        //Setup. Declare ticks & ticks to degrees
        /* ticksPerDegree is circumference of the circle determined by the distances between the
           adjacent wheels on the robot, turned to motor ticks, then divided by 60 */
        double ticksPerDegree = 20.5;
        double _ticks = degrees * ticksPerDegree; //precision
        int ticks = (int)_ticks;

        telemetry.addData("Degrees: ", degrees);
        telemetry.addData("Ticks: ", ticks);
        telemetry.update();

        //reset motor encoders to be safe
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set motor ticks and run to target
        robot.rightFrontMotor.setTargetPosition(-ticks);
        robot.rightBackMotor.setTargetPosition(-ticks);
        robot.leftFrontMotor.setTargetPosition(ticks);
        robot.leftBackMotor.setTargetPosition(ticks);

        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Drivetrain: ", "Activating");
        telemetry.addData("Power: ", power);
        telemetry.update();

        robot.rightFrontMotor.setPower(-power);
        robot.rightBackMotor.setPower(-power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);

        //stop at end of turn
        while(robot.leftFrontMotor.isBusy()) {
            telemetry.addData("rightFront position: ", robot.rightFrontMotor.getCurrentPosition());
            telemetry.addData("leftFront position: ", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("rightBack position: ", robot.rightBackMotor.getCurrentPosition());
            telemetry.addData("leftBack position: ", robot.leftBackMotor.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drivetrain: ", "Done");
        telemetry.update();
        noDrive(robot);
    }

    //place block (for vuforia)
    public void placeBlock(RelicRecoveryHardware robot, long breakTime) {
        sayAndPause("Arm: ", "Lowering", breakTime);
        lowerArm(robot, 3150);
        sleep(1250);
        openingClaw(robot);
        sayAndPause("Arm: ", "Raising", breakTime);
        raiseArm(robot, 3150);
        driveForward(.5, 10, robot);
        driveBackward(0.5, 10, robot);
        driveForward(.5, 10, robot);
    }
    public void redVuforia(RelicRecoveryVuMark vuMark, RelicRecoveryHardware robot) {
        if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            driveForward(0.5, 4, robot);
        }
        else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            driveBackward(-0.5, 4, robot);
        }
        else {
            //Do nothing
        }

    }

    public void blueVuforia(RelicRecoveryVuMark vuMark, RelicRecoveryHardware robot) {
        if(vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
            driveForward(0.5, 4, robot);
        }
        else if(vuMark.equals(RelicRecoveryVuMark.LEFT)) {
            driveBackward(-0.5, 4, robot);
        }
        else {
            //Do nothing
        }

    }

    public void sayAndPause(String title, String caption, long pause) {
        telemetry.addData(title, caption);
        telemetry.update();
        sleep(pause);
    }

    public void extendLeadScrew(RelicRecoveryHardware robot) {
        telemetry.addData("Lead Screw: ", "Moving");
        telemetry.update();
        robot.leadScrew.setPower(-1);
        sleep(5000);
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

    public void blueBallKnock(RelicRecoveryHardware robot) {
        robot.ballSwivel.setPosition(1);
        robot.ballLower.setPosition(0.1);
        robot.ballSwivel.setPosition(0.5);
        sleep(250);
        robot.ballLower.setPosition(0.6);
        sleep(750);
        if (robot.colorSensor.getNormalizedColors().blue < robot.colorSensor.getNormalizedColors().red) {
            sayAndPause("Ball Color: ", "Red", 250);
            robot.ballSwivel.setPosition(0.8);
        }
        else if(robot.colorSensor.getNormalizedColors().red<robot.colorSensor.getNormalizedColors().blue) {
            sayAndPause("Ball Color: ", "Blue", 250);
            robot.ballSwivel.setPosition(0.2);
        }
        else {
            //Do nothing
        }
        sleep(250);
        robot.ballLower.setPosition(0.0);
        sleep(250);
        robot.ballSwivel.setPosition(0);
        sleep(250);
    }

    // Same code as blueBallKnock, except knocking ball position reversed.
   public void redBallKnock(RelicRecoveryHardware robot) {
       sayAndPause("Red Ball Knock Starting", "", 250);
       robot.ballSwivel.setPosition(1);
       robot.ballLower.setPosition(0.1);
       robot.ballSwivel.setPosition(0.5);
       sleep(250);
       robot.ballLower.setPosition(0.6);
       sleep(750);
       if (robot.colorSensor.getNormalizedColors().blue < robot.colorSensor.getNormalizedColors().red) {
           sayAndPause("Ball Color: ", "Red", 250);
           robot.ballSwivel.setPosition(0.2);
       }
       else if(robot.colorSensor.getNormalizedColors().red<robot.colorSensor.getNormalizedColors().blue) {
           sayAndPause("Ball Color: ", "Blue", 250);
           robot.ballSwivel.setPosition(0.8);
       }
       else {
           //Do nothing
       }
       sleep(250);
       robot.ballLower.setPosition(0.0);
       sleep(250);
       robot.ballSwivel.setPosition(0);
       sleep(250);
   }

    public void lowerArm(RelicRecoveryHardware robot, int height) {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(robot.arm.getCurrentPosition() - height);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);

    }
}