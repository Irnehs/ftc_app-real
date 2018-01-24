package org.firstinspires.ftc.robotcontroller.internal.OpModes;

/**
 * Created by kevinrockwell on 1/23/18.
 */

public class TestJewelArm extends RelicBaseAuto {


    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()) {

            boolean ballLower = gamepad1.a;
            boolean ballRaise = gamepad1.y;
            boolean ballSwivelRight = gamepad1.b;
            boolean ballSwivelLeft = gamepad1.x;

            if(ballLower){
                robot.ballLower.setPosition(0);
            }
            else if(ballRaise){
                robot.ballLower.setPosition(0.5);
            }
            else if(ballSwivelRight){
                robot.ballSwivel.setPosition(0.2);
                sleep(250);
                robot.ballSwivel.setPosition(0.5);
            }
            else if(ballSwivelLeft){
                robot.ballSwivel.setPosition(0.8);
                sleep(250);
                robot.ballSwivel.setPosition(0.5);
            }

        }

    }
}
