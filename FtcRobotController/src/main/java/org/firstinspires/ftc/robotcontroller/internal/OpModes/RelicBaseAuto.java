package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Feklaar on 12/9/2017.
 */
@Disabled
abstract class RelicBaseAuto extends LinearOpMode {
    /* Declare OpMode members. */
    RelicRecoveryHardware robot = new RelicRecoveryHardware();   // Use a Pushbot's hardware
    String Version = "0.0.3";

    public void driveForward(double power, long time) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(power);
        robot.leftFrontMotor.setPower(power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
    }
    public void driveBackward(double power, long time) {
        driveForward(-power, time);
    }
    public void driveLeft(double power, long time) {
        robot.rightFrontMotor.setPower(power);
        robot.rightBackMotor.setPower(-power);
        robot.leftFrontMotor.setPower(-power);
        robot.leftBackMotor.setPower(power);
        sleep(time);
    }

    public void driveRight(double power, long time) {
        driveLeft(-power, time);
    }

    public void noDrive() {
        driveForward(0,0);
    }

    public void firstSteps(boolean isBlue) {
        long horizontalTime = 1000;
        long verticalTime = 250;
        //Close claws

        //lift arm 1 step, extend arm

        //drive left
        driveLeft(1,horizontalTime);

        //Scan color, jewel arm down
        /*double ballRed = robot.colorSensor.red();
        double ballBlue = robot.colorSensor.blue();
        boolean ballIsBlue;

        //Drive forward/Backwards (color dependent)
        if (ballRed > 0 || ballBlue > 0) {
            if (ballRed > ballBlue)
                ballIsBlue = false;
            else
                ballIsBlue = true;

            if (ballIsBlue == isBlue) {
                driveBackward(1, verticalTime);
                //armup()
                driveForward(1, verticalTime);
            }
            else {
                driveForward(1, verticalTime);
                //armup()
                driveBackward(1, verticalTime);
            }

        }
        */
        //scan Vuforia (might need to adjust phone or robot position)

        //Drive right

        /*Start of Non-generic code*/

        //blue high
    }
}
