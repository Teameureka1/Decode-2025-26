package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "EncoderInchesAuto")
public class EncoderInchesAuto extends LinearOpMode {

    DcMotor fl, bl, fr, br;
    IMU imu;

    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double TICKS_PER_REV = 5203; // SWYFT v2 motor
    static final double MM_PER_INCH = 25.4;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive motors
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

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("Robot Ready");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {
            moveForward(12, 0.5);
            strafeRight(10, 0.4);
            moveBackward(6, 0.3);
            strafeLeft(8, 0.6);
            turnToAngle(90);

            telemetry.addLine("Finished Autonomous");
            telemetry.update();
        }
    }

    private int inchesToTicks(double inches) {

        double rotations = inches / (Math.PI * WHEEL_DIAMETER_INCHES);
        return (int)(rotations * TICKS_PER_REV);
    }

    private void moveForward(double inches, double power) {
        int ticks = inchesToTicks(inches);
        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);
        runToPosition(power);
        hardStop();
    }

    private void moveBackward(double inches, double power) {
        int ticks = inchesToTicks(inches);
        fl.setTargetPosition(fl.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);
        runToPosition(power);
        hardStop();
    }

    private void strafeRight(double inches, double power) {
        int ticks = inchesToTicks(inches * 0.707);
        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);
        runToPosition(power);
        hardStop();
    }

    private void strafeLeft(double inches, double power) {
        int ticks = inchesToTicks(inches * 0.707);
        fl.setTargetPosition(fl.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);
        runToPosition(power);
        hardStop();
    }

    private void runToPosition(double power) {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        while(opModeIsActive() &&
                (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.update();
        }
    }

    private void hardStop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        sleep(50);
    }

    private void turnToAngle(double targetAngle) {
        double power = 0.5;
        double tolerance = 3; // degrees

        while(opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngle - heading;

            while(error > 180) error -= 360;
            while(error < -180) error += 360;

            if(Math.abs(error) <= tolerance) break;

            double turnPower = Math.signum(error) * power;
            fl.setPower(turnPower);
            bl.setPower(turnPower);
            fr.setPower(-turnPower);
            br.setPower(-turnPower);
        }
        hardStop();
    }
}