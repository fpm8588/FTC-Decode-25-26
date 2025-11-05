/*package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//RoadRunner Specific
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR Specific
import org.firstinspires.ftc.teamcode.Base.AutonomousBaseV1;
import org.firstinspires.ftc.teamcode.Base.RobotRunType;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Tester Auto", group = "Base")
public class testerAuto extends AutonomousBaseV1 {

    @Override
    public void runOpMode() {
       // arm am = new arm(this);
        //set up robot for teleop
        initRobotV2(RobotRunType.AUTONOMOUS);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
        }
        MecanumDrive drive;
        Pose2d initialPose;
        {
            // instantiate your MecanumDrive at a particular pose.
            initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
            drive = new MecanumDrive(hardwareMap, initialPose);

            //vision output position
            int visionOutputPosition = 1;


        }
        waitForStart();


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .waitSeconds(2)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
    }
}
*/