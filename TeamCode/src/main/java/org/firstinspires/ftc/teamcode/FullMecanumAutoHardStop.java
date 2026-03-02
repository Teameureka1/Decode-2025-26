package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "FullMecanumAutoHardStop")
public class FullMecanumAutoHardStop extends LinearOpMode {

    DcMotor fl, bl, fr, br;
    IMU imu;

    static final double WHEEL_DIAMETER_INCHES = 4.0;

    @Override
    public void runOpMode() {

        // --- Hardware setup ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        // --- SEQUENTIAL MOVEMENTS WITH HARD STOPS ---
        moveForward(0.5, 12);
        strafeRight(0.4, 10);
        turnToAngle(90);
        moveBackward(0.3, 6);
        strafeLeft(0.6, 8);
        moveForward(0.7, 24);

        sleep(500);
    }

    // --- Drive forward ---
    public void moveForward(double power, double inches) {
        int ticks = inchesToTicks(fl, inches);

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);

        runToPosition(power);
        hardStop();
    }

    // --- Drive backward ---
    public void moveBackward(double power, double inches) {
        int ticks = inchesToTicks(fl, inches);

        fl.setTargetPosition(fl.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);

        runToPosition(power);
        hardStop();
    }

    // --- Strafe right (corrected distance) ---
    public void strafeRight(double power, double inches) {
        int ticks = inchesToTicks(fl, inches * 0.707);

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);

        runToPosition(power);
        hardStop();
    }

    // --- Strafe left (corrected distance) ---
    public void strafeLeft(double power, double inches) {
        int ticks = inchesToTicks(fl, inches * 0.707);

        fl.setTargetPosition(fl.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);

        runToPosition(power);
        hardStop();
    }

    // --- Run to target position ---
    private void runToPosition(double power) {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        while (opModeIsActive() &&
                (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            telemetry.addData("Status", "Moving...");
            telemetry.update();
        }

        // Stop motors for hard stop
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Turn to absolute angle using IMU ---
    private void turnToAngle(double targetAngle) {
        double power = .6;
        double tolerance = 3;
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngle - heading;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) <= tolerance) break;

            if (System.currentTimeMillis() - startTime > 1000) break;

            double turn = Math.signum(error) * power;
            fl.setPower(turn);
            bl.setPower(turn);
            fr.setPower(-turn);
            br.setPower(-turn);

            telemetry.addData("Heading", heading);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Hard stop after turn
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        sleep(50); // brief pause to stabilize
    }

    // --- Hard stop helper ---
    private void hardStop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        sleep(50); // brief pause to let robot settle
    }

    // --- Convert inches to encoder ticks dynamically from motor ---
    private int inchesToTicks(DcMotor motor, double inches) {
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        double rotations = inches / (Math.PI * WHEEL_DIAMETER_INCHES);
        return (int)(rotations * ticksPerRev);
    }
}