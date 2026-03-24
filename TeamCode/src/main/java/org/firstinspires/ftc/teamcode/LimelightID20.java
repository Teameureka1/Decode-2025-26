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

import java.util.List;

@TeleOp(name = "LimelightID20")
public class LimelightID20 extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx intake, kicker, launcher, launcher2;
    private Limelight3A limelight;

    private final double kP = 0.03;
    private final double minPower = 0.09;
    private final double maxPower = 0.38;
    private final double deadzone = 0.8;

    // === ONE‑TAP TRIPLE SHOT VARIABLES ===
    private boolean tripleShotActive = false;
    private long tripleShotStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backRightMotor  = hardwareMap.get(DcMotor.class, "br");

        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        kicker   = hardwareMap.get(DcMotorEx.class, "kicker");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        Servo wall = hardwareMap.get(Servo.class, "wall");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(55, 0, 0, 14.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        telemetry.addLine("Ready — Hold Left Trigger to Auto‑Rotate");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.7;

            // === THROTTLE ===
            double throttle = gamepad1.right_trigger;
            double speedMultiplier = 0.3 + (0.7 * throttle);

            // === INTAKE ===
            double intakeY = gamepad2.left_stick_y;

            if (intakeY > 0.1) {
                intake.setPower(-1);
                kicker.setPower(-1);
            } else if (intakeY < -0.1) {
                intake.setPower(1);
                kicker.setPower(1);
            } else if (!tripleShotActive) {
                intake.setPower(0);
                kicker.setPower(0);
            }

            // === WALL SERVO ===
            if (gamepad2.a) wall.setPosition(.15);
            if (gamepad2.y) wall.setPosition(.32);

            // === LAUNCHER ===
            if (gamepad2.right_trigger > 0.35) {
                launcher.setVelocity(1275);
                launcher2.setVelocity(1275);
            } else {
                launcher.setVelocity(1000);
                launcher2.setVelocity(1000);
            }

            // === ONE‑TAP TRIPLE SHOT WITH SPIN-UP WAIT ===

            // Start burst when B is pressed
            if (gamepad2.x && !tripleShotActive) {
                tripleShotActive = true;
                tripleShotStartTime = System.currentTimeMillis();

                // Spin launcher to firing speed
                launcher.setVelocity(1275);
                launcher2.setVelocity(1275);
            }


            if (tripleShotActive) {

                launcher.setVelocity(1275);
                launcher2.setVelocity(1275);
                long elapsed = System.currentTimeMillis() - tripleShotStartTime;


                if (elapsed > 1000 && elapsed < 1300) {

                    kicker.setPower(1);
                }

                // Stop after ~1 second of firing
                if (elapsed > 1800) {
                    kicker.setPower(0);
                    tripleShotActive = false;
                }
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
