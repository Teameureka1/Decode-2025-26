package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name = "EncoderInchesAuto")
public class EncoderInchesAuto extends LinearOpMode {

    public DcMotorEx odometerForward;
    public DcMotorEx odometerStrafe;
    DcMotorEx fl, bl, fr, br;
    IMU imu;

    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double TICKS_PER_REV = 5203; // SWYFT v2 motor
    static final double MM_PER_INCH = 25.4;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive motors
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        br = hardwareMap.get(DcMotorEx.class, "br");

        odometerForward = fl;
        odometerStrafe = bl;

        fl.getCurrentPosition();
        bl.getCurrentPosition();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("Robot Ready");
        telemetry.update();
        waitForStart();

        if(opModeIsActive()) {


            telemetry.addLine("Finished Autonomous");
            while(opModeIsActive()) {
                telemetry.addData("Forward Ticks ", fl.getCurrentPosition());
                telemetry.addData("Strafe Ticks", bl.getCurrentPosition());
                telemetry.update();
            }
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
    }

    private void hardStop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        sleep(50);
    }

    private void turnToAngle(double targetAngle) {
        double power = 0.4;
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