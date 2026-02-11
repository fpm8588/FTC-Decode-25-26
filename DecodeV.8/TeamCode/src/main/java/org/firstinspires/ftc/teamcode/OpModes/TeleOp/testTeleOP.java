package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Base.AutonomousBaseV1;
import org.firstinspires.ftc.teamcode.Base.RobotRunType;

@TeleOp (name = "Test TeleOp" )
public class testTeleOP extends AutonomousBaseV1 {

    @Override
    public void runOpMode() {

        //set up robot for teleop
        initRobotV2(RobotRunType.AUTONOMOUS);
        double collectorPow;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
        }

        while (opModeIsActive()) {

            /** //////////////////////////////////
             ///////////////Driver 1///////////////
             ////////////////////////////////// **/

            //control drive train
            FieldCentricDrive();
            //allows reset of gyroscope when aligned with driver preference
            if (gamepad1.a && gamepad1.b) {
                resetAngle();
            }
            /** ////////////////////////////////
             * ///////////Driver 2//////////////
             ///////////////////////////////**/

            /** Scoring System **/
            //The Dozer
            if (gamepad2.right_trigger > 0.1) {
                spinOne.setVelocity(1400);
                //spinTwo.setVelocity(2800);
            }
            else if (gamepad2.left_trigger > 0.1) {
                spinOne.setVelocity(1100);
                //spinTwo.setVelocity(2800);
            }
            else {
                spinOne.setVelocity(0);
                //spinTwo.setVelocity(0);
            }

            /** Intake Systems **/
            // The Cloaker
            if (gamepad2.dpad_left) {
                inOne.setPower(-0.6);
                inTwo.setPower(0.6);
            }
            else {
                inOne.setPower(0);
                inTwo.setPower(0);
            }
            if (gamepad2.x) {
                lift.setPosition(1);
            }
            else if (gamepad2.a) {
                lift.setPosition(0.5);
            }
            else if (gamepad2.b) {
                lift.setPosition(0);
            }
            if(gamepad2.left_bumper) {
                sortOne.setPosition(1);
                sortTwo.setPosition(0);
            }
            else if (gamepad2.right_bumper) {
                sortTwo.setPosition(1);
                sortOne.setPosition(0);
            }

        }
    }
}
