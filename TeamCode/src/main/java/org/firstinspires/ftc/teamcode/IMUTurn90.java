package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

// Set brake mode to prevent coasting
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            turnToAngle(90, .2);  // Turn 90 degrees at 40% max power

            sleep(1000);
        }
    }
    private void turnToAngle(double targetAngle, double maxPower) {

        double kP = 0.003;   // proportional
        double kD = 0.002;   // damping
        double tolerance = 6; // degrees

        double lastError = 0;
        long lastTime = System.currentTimeMillis();
        long insideStartTime = 0;

        while (opModeIsActive()) {

            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetAngle - currentAngle;

            // Wrap to -180 -> 180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Derivative
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;
            double derivative = (error - lastError) / deltaTime;

            double power = (kP * error) + (kD * derivative);
            power = Range.clip(power, -maxPower, maxPower);

            // Smooth slow-down near target
            if (Math.abs(error) < 15) power *= 0.6;

            // Prevent tiny twitching
            if (Math.abs(power) < 0.22) power = 0;

            // Apply power to mecanum wheels
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            // --- Stop if within tolerance for 200ms ---
            if (Math.abs(error) <= tolerance) {
                if (insideStartTime == 0) insideStartTime = System.currentTimeMillis();
                if (System.currentTimeMillis() - insideStartTime >= 200) break;
            } else {
                insideStartTime = 0;
            }

            lastError = error;
            lastTime = currentTime;

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Current", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}