package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "IMU 90 Degree Turn", group = "Autonomous")
public class IMUTurn90 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    @Override
    public void runOpMode() {

        // Motor setup
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        // Reverse left side if needed
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // IMU setup (Control Hub built-in)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        waitForStart();

        if (opModeIsActive()) {

            // Reset yaw to zero before turning
            imu.resetYaw();

            turnToAngle(90, .4);  // Turn 90 degrees at 40% max power

            sleep(1000);
        }
    }

    private void turnToAngle(double targetAngle, double maxPower) {

        double kP = 0.006;  // Proportional constant (tune this!)
        double tolerance = 3.0; // degrees

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            double error = targetAngle - currentAngle;

            if (Math.abs(error) <= tolerance) {
                break;
            }

            double power = Range.clip(error * kP, -maxPower, maxPower);

            // Tank turn
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Current", currentAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Stop motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }
}