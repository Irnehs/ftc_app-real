package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RelicRecoveryHardware
{
    /* Public OpMode members. */
    // Create motor object out of DcMotor class
    //public DcMotor arm = null;

    //Create servo object out of Servo class
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    //Declare Hardware
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo ballLower;
    public Servo ballSwivel;

    public DcMotor arm;
    public DcMotor leadScrew;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    //public GyroSensor gyroSensor;
    public NormalizedColorSensor colorSensor;
    //public DistanceSensor distanceSensor;

    /* Constructor */
    public RelicRecoveryHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        arm = hwMap.dcMotor.get("arm");
        leadScrew = hwMap.dcMotor.get("leadScrew");
        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        leftBackMotor = hwMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hwMap.dcMotor.get("rightBackMotor");

        // Define and initialize ALL installed servos.
        rightClaw = hwMap.servo.get("rightClaw");
        leftClaw = hwMap.servo.get("leftClaw");
        ballLower = hwMap.servo.get("ballLower");
        ballSwivel = hwMap.servo.get("ballSwivel");

        //define sensors
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        //distanceSensor = hwMap.get(DistanceSensor.class, "colorDistanceSensor");
        //gyroSensor = hwMap.gyroSensor.get("gyroSensor");

        //Sets spin directions to make writing power easier
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power to prevent it from accidentally turning on
        arm.setPower(0);
        leadScrew.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Set all motors' run modes.
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leadScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

