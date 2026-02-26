package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumRobot {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public IMU imu;

    private final double kP = 0.007;
    private final double kD = 0.002;
    private final double tolerance = 2.0; // degrees

    public MecanumRobot(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, IMU imuSensor) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        imu = imuSensor;

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // BRAKE mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // --- Absolute PD Turn ---
    public void turnToAngle(double targetAngle, double maxPower) {

        double lastError = 0;
        long lastTime = System.currentTimeMillis();

        while (true) {

            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double error = targetAngle - currentAngle;

            // Wrap to -180 -> 180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;
            double derivative = (error - lastError) / deltaTime;

            double power = (kP * error) + (kD * derivative);
            power = Range.clip(power, -maxPower, maxPower);

            if (Math.abs(error) < 15) power *= 0.6;

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            if (Math.abs(error) <= tolerance) break;

            lastError = error;
            lastTime = currentTime;
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    // --- New: Relative Turn ---
    public void turnBy(double deltaAngle, double maxPower) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = currentHeading + deltaAngle;

        // Wrap to -180 -> 180
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;

        turnToAngle(targetHeading, maxPower);
    }
}