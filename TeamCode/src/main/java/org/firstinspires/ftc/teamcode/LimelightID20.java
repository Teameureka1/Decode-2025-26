package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configuration.Config;

import java.util.List;

@TeleOp(name = "LimelightID20")
public class LimelightID20 extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx intake, kicker, launcher, launcher2;
    private Limelight3A limelight;

    boolean reverseBurstActive = false;
    double reverseStartTime = 0;
    boolean intakeWasUp = false;

    private final double kP = 0.3; // was .035
    private final double minPower = .3; // was .23
    private final double maxPower = .5; // was .4
    private final double deadzone = 0.6; // was .8

    ElapsedTime runTime = new ElapsedTime();

    Config robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Config(this);
        robot.init();


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.7;

            // === THROTTLE ===
            double throttle = gamepad1.right_trigger;
            double speedMultiplier = 0.3 + (0.7 * throttle);

            // === INTAKE ===
            double intakeY = -gamepad2.left_stick_y;

            boolean intakeUp = intakeY < -0.1;

            if (!intakeUp && intakeWasUp && !reverseBurstActive) {
                reverseBurstActive = true;
                reverseStartTime = runTime.milliseconds();
            }

            if (reverseBurstActive) {
                if (runTime.milliseconds() - reverseStartTime < 150) {
                    intake.setPower(1);
                } else {
                    reverseBurstActive = false;
                    intake.setPower(0);
                    kicker.setPower(0);
                }
            }
            // Normal control
            else {
                if (intakeY < -0.1) {

                    intake.setPower(-1);
                    kicker.setPower(-1);
                } else if (intakeY > 0.1) {
                    intake.setPower(1);
                    kicker.setPower(1);
                } else {
                    intake.setPower(0);
                    kicker.setPower(0);
                }
            }
            intakeWasUp = intakeUp;

            // === WALL SERVO ===
            if (gamepad2.a) robot.wall.setPosition(.15);
            if (gamepad2.y) robot.wall.setPosition(.32);

            // === LAUNCHER CONTROL ===
            if (gamepad2.right_trigger > 0.35) {
                launcher.setVelocity(1300);
                launcher2.setVelocity(1300);
            } else if (gamepad2.left_trigger > 0.35) {
                launcher.setVelocity(2000);
                launcher2.setVelocity(2000);
            } else {
                launcher.setVelocity(1000);
                launcher2.setVelocity(1000);
            }

            if (gamepad2.a && gamepad2.b && gamepad2.y && gamepad2.x) {
                requestOpModeStop();
            }

            // === AUTO ROTATE ===
            if (gamepad1.left_trigger > 0.1) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 20) {

                            double tx = fr.getTargetXDegrees();

                            if (Math.abs(tx) < deadzone) {
                                rotation = 0;
                            } else {
                                rotation = tx * kP;

                                if (Math.abs(rotation) < minPower)
                                    rotation = Math.signum(rotation) * minPower;

                                if (Math.abs(rotation) > maxPower)
                                    rotation = Math.signum(rotation) * maxPower;
                            }
                            break;
                        }
                    }
                }
            }

            // === MECANUM DRIVE ===
            double fl = (y + x + rotation) * speedMultiplier;
            double bl = (y - x + rotation) * speedMultiplier;
            double fr = (y - x - rotation) * speedMultiplier;
            double br = (y + x - rotation) * speedMultiplier;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br)));

            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            frontLeftMotor.setPower(fl);
            backLeftMotor.setPower(bl);
            frontRightMotor.setPower(fr);
            backRightMotor.setPower(br);
        }
    }
}