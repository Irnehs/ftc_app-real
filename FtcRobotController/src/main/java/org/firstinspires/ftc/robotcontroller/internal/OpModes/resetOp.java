package org.firstinspires.ftc.robotcontroller.internal.OpModes;

import org.firstinspires.ftc.robotcontroller.internal.OpModes.BaseOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Feklaar on 1/4/2018.
 */
@TeleOp(name="ResetOp", group="TeleOp")
public class resetOp extends BaseOpMode {
    RelicRecoveryHardware robot = new RelicRecoveryHardware();
    String Version = "0.0.3";

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        /*INITIALIZATION OF ALL VARIABLES*/

        /*Glyph Arm Variables*/

        int rotation = 1120;
        int halfRotation = rotation / 2;
        int quarterRotation = halfRotation / 2;
        double armPosition = 1;
        double armSpeed = 0.2;

        waitForStart();
        while(opModeIsActive()) {
            boolean armUp = gamepad1.a;
            boolean armDown = gamepad1.b;
            boolean leadScrewIn = gamepad2.a;
            boolean leadScrewOut = gamepad2.b;
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Arm control
            if(armUp)
                robot.arm.setPower(1);
            else if(armDown)
                robot.arm.setPower(-0.1);
            else
                robot.arm.setPower(0);

            //Lead screw control
            if(leadScrewIn)
                robot.leadScrew.setPower(-0.1);
            else if(leadScrewOut)
                robot.leadScrew.setPower(0.1);
            else
                robot.leadScrew.setPower(0);
        }



    }
}
