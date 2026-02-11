package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class RobotHardware extends RobotBaseV1 {

    // Declares drive motors
    protected DcMotorEx leftFront;
    protected DcMotorEx leftBack;
    protected DcMotorEx rightFront;
    protected DcMotorEx rightBack;
    // Declares servos for intake
    protected DcMotor inOne;
    protected DcMotor inTwo;
    protected DcMotorEx spinOne;
    protected DcMotorEx spinTwo;
    protected Servo lift;
    protected Servo sortOne;
    protected Servo sortTwo;
    public static double armTarget;

    // declares gyro and gyro variables
    protected IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation angles;
    protected double globalAngle;
    protected int heading;

    //final variables for moving robot to distance
    protected final double WHEEL_DIAMETER = 4;
    protected final double WHEEL_CIRC = WHEEL_DIAMETER * Math.PI;
    protected final double ultraplanetary_PPR = 537.7;
    protected final double neverest20ppr = 537.6;
    protected final double DRIVE_GEAR_RATIO = 1;
    protected final int ARM_RATIO = 80;
    protected final double armPPR = 384.5;
    private final double CLAW_GRAB_POSITION = 0.5;
    private final double CLAW_RELEASE_POSITION = 0.0;

    protected void initRobotV2(RobotRunType robotRunType) {
        // set up drive motors
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("lb");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rb");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set up linear slides
        spinOne = (DcMotorEx) hardwareMap.dcMotor.get("spinOne");
        spinTwo = (DcMotorEx) hardwareMap.dcMotor.get("spinTwo");

        inOne = hardwareMap.dcMotor.get("inOne");
        inTwo = hardwareMap.dcMotor.get("inTwo");

        spinTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinOne.setDirection(DcMotorSimple.Direction.REVERSE);
        spinTwo.setDirection(DcMotorEx.Direction.REVERSE);
        lift = hardwareMap.servo.get("lift");
        sortOne = hardwareMap.servo.get("sortOne");
        sortTwo = hardwareMap.servo.get("sortTwo");


        final double COUNTS_PER_MOTOR_REV = 84; // For REV HD Hex Motor 20:1
        final double MAX_RPM = 2000;
        double desiredRPM = 1500;
        double ticksPerSecond = (desiredRPM / 60.0) * COUNTS_PER_MOTOR_REV;

        // initialize gyro if starting in autonomous
        if (robotRunType == RobotRunType.AUTONOMOUS) {
            imu = hardwareMap.get(BHI260IMU.class, "imu");

            imu.initialize(
                    new IMU.Parameters(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                            )
                    )
            );

            telemetry.addData("Mode", "Calibrating");
            telemetry.update();
            telemetry.addData("Mode", "waiting for start");
            telemetry.update();
        }
    }

    protected void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    protected void stopDrive(){setDrivePower(0, 0, 0, 0);}

    protected void MecanumFormula(double forward, double strafe, double turning) {
        double lfPower, lbPower, rfPower, rbPower;
        double max;

        lfPower = lbPower = rfPower = rbPower = 0;

        //forward/back
        lfPower += forward;
        lbPower += forward;
        rfPower += forward;
        rbPower += forward;

        //strafing
        lfPower += strafe;
        lbPower -= strafe;
        rfPower -= strafe;
        rbPower += strafe;

        //turning
        lfPower += turning;
        lbPower += turning;
        rfPower -= turning;
        rbPower -= turning;

        max = Math.abs(lfPower);
        if (Math.abs(lbPower) > max) max = Math.abs(lbPower);
        if (Math.abs(rfPower) > max) max = Math.abs(rfPower);
        if (Math.abs(rbPower) > max) max = Math.abs(rbPower);

        if (max > 1) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        setDrivePower(rfPower, lfPower, rbPower, lbPower);
    }

    // ===== FIXED FIELD-CENTRIC DRIVE =====
    protected void FieldCentricDrive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y; // forward is negative

        double v = Math.hypot(x, y);
        double theta = Math.atan2(y, x);

        double heading = Math.toRadians(getGlobal() % 360);

        // rotate joystick vector by -heading
        drive(theta - heading, v, gamepad1.right_stick_x);
    }

    protected static class Wheels {
        public double lf, lr, rf, rr;

        public Wheels(double lf, double rf, double lr, double rr) {
            this.lf = lf;
            this.rf = rf;
            this.lr = lr;
            this.rr = rr;
        }
    }

    public void setArmTarget(double b) {
        armTarget = b;
    }

    protected Wheels getWheels(double direction, double velocity, double rotationVelocity) {
        final double vd = velocity;
        final double td = direction;
        final double vt = rotationVelocity;

        double s = Math.sin(td + Math.PI / 4.0);
        double c = Math.cos(td + Math.PI / 4.0);
        double m = Math.max(Math.abs(s), Math.abs(c));
        s /= m;
        c /= m;

        final double v1 = vd * s + vt;
        final double v2 = vd * c - vt;
        final double v3 = vd * c + vt;
        final double v4 = vd * s - vt;

        double scale = ma(1.0, v1, v2, v3, v4);

        return new Wheels(v1 / scale, v2 / scale, v3 / scale, v4 / scale);
    }

    protected static double ma(double... xs) {
        double ret = 0.0;
        for (double x : xs) ret = Math.max(ret, Math.abs(x));
        return ret;
    }

    protected void drive(double direction, double velocity, double rotationVelocity) {
        Wheels w = getWheels(direction, velocity, rotationVelocity);
        leftFront.setPower(w.lf);
        rightFront.setPower(w.rf);
        leftBack.setPower(w.lr);
        rightBack.setPower(w.rr);
    }

    protected double getGlobal() {
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        telemetry.addData("global theta: ", globalAngle);
        telemetry.addData("imu theta: ", angles.firstAngle);
        telemetry.update();
        return globalAngle;
    }
}