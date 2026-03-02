package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "ControlHubIMUTurn90", group = "Autonomous")
public class ControlHubIMUTurn90 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    @Override
    public void runOpMode() {

        // Motors
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // BRAKE mode so robot doesn't coast
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        if (opModeIsActive()) {

            imu.resetYaw();

            turnToAngle(90);

            sleep(2000);
        }
    }

    private void turnToAngle(double targetAngle) {

        double power = 0.4;
        double tolerance = 7; // bigger tolerance prevents shaking
        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            double error = targetAngle - heading;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Stop if close enough
            if (Math.abs(error) <= tolerance) {
                break;
            }

            // Safety timeout (2 seconds)
            if (System.currentTimeMillis() - startTime > 1475) {
                break;
            }

            double turn = Math.signum(error) * power;

            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);

            telemetry.addData("Heading", heading);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // HARD STOP
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}