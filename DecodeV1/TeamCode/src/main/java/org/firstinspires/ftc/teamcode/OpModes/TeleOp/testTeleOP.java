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
                spinOne.setPower(1);
                spinTwo.setPower(-1);
            } else {
                spinOne.setPower(0);
                spinTwo.setPower(0);
            }
            //The Tazer
            if (gamepad2.left_bumper) {
                grab();
            }
            if (gamepad2.right_bumper) {
                release();
            }
            /** Intake Systems **/
            // The Cloaker

            if (gamepad2.x) {
                wristR.setPosition(0);
            }
            if (gamepad2.dpad_left) {
                wristR.setPosition(1);
            }
            // The Medic
            if (gamepad2.a) {
                thePinch.setPosition(0.75);
            }
            if (gamepad2.dpad_down) {
                thePinch.setPosition(0.5);
            }
        }
    }
}
