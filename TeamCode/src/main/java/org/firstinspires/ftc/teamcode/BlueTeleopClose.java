package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.Config;

import java.util.List;

@TeleOp(name = "BlueTeleopClose")
public class BlueTeleopClose extends LinearOpMode {

    private Config robot;

    private Limelight3A limelight;
    private Servo vision, vision1, wall;

    private DcMotorEx launcher, launcher2;

    // ================= LIGHT VALUES =================
    private final double OFF = 0.0;
    private final double GREEN = 0.25;
    private final double YELLOW = 0.45;
    private final double ORANGE = 0.3175;
    private final double PURPLE = 0.7;

    // ================= ARTIFACT DETECTION (ENCODER BASED) =================
    private int artifactCount = 0;

    private int lastIntakePos = 0;
    private double lastIntakeVel = 0;

    private boolean inCooldown = false;
    private double cooldownStart = 0;
    private final double COOLDOWN_MS = 300;

    // Spike tuning (ENCODER BASED — MAIN DETECTION)
    private final int ENCODER_DROP_THRESHOLD = 80;   // ticks drop per loop
    private final double VELOCITY_DROP_THRESHOLD = 150;

    // ================= LIMELIGHT =================
    private final double kP = 0.27;
    private final double minPower = 0.4;
    private final double maxPower = 0.55;
    private final double deadzone = 0.8;

    @Override
    public void runOpMode() {

        robot = new Config(this);
        robot.init();

        vision = hardwareMap.get(Servo.class, "vision");
        vision1 = hardwareMap.get(Servo.class, "vision1");
        wall = hardwareMap.get(Servo.class, "wall");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        // initialize encoder baseline
        lastIntakePos = robot.intake.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.75;

            double speed = 0.3 + (0.7 * gamepad1.right_trigger);

            // ================= INTAKE =================
            double intakeInput = -gamepad2.left_stick_y;

            if (intakeInput < -0.1) {
                robot.intake.setPower(-0.585);
                robot.kicker.setPower(-1);
            } else if (intakeInput > 0.1) {
                robot.intake.setPower(0.585);
                robot.kicker.setPower(1);
            } else {
                robot.intake.setPower(0);
                robot.kicker.setPower(0);
            }

            // ================= WALL =================
            if (gamepad2.a) wall.setPosition(0.15);
            if (gamepad2.y) wall.setPosition(0.32);

            // ================= LAUNCHER =================
            if (gamepad2.right_trigger > 0.35) {
                launcher.setVelocity(1260);
                launcher2.setVelocity(1260);
            } else if (gamepad2.left_trigger > 0.35) {
                launcher.setVelocity(1600);
                launcher2.setVelocity(1600);
            } else {
                launcher.setVelocity(1140);
                launcher2.setVelocity(1140);
            }

            // ================= LIMELIGHT =================
            boolean locked = false;

            if (gamepad1.left_trigger > 0.1) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 20) {

                            double tx = fr.getTargetXDegrees();

                            if (Math.abs(tx) < deadzone) {
                                rotation = 0;
                                locked = true;
                            } else {
                                rotation = tx * kP;

                                if (Math.abs(rotation) < minPower)
                                    rotation = Math.signum(rotation) * minPower;

                                if (Math.abs(rotation) > maxPower)
                                    rotation = Math.signum(rotation) * maxPower;
                            }
                        }
                    }
                }
            }

            // ================= 🔥 ENCODER TRANSFER DETECTOR (RELIABLE) =================
            int intakePos = robot.intake.getCurrentPosition();
            double intakeVel = robot.intake.getVelocity();

            int deltaPos = intakePos - lastIntakePos;

            boolean intakeRunning = Math.abs(intakeInput) > 0.1;

            // MAIN DETECTION:
            boolean encoderSpike = deltaPos < -ENCODER_DROP_THRESHOLD;
            boolean velocitySpike = (lastIntakeVel - intakeVel) > VELOCITY_DROP_THRESHOLD;

            boolean transferDetected = intakeRunning && (encoderSpike || velocitySpike);

            // cooldown prevents double counting
            if (inCooldown) {
                if (getRuntime() * 1000 - cooldownStart > COOLDOWN_MS) {
                    inCooldown = false;
                }
            }

            if (transferDetected && !inCooldown) {
                artifactCount++;
                if (artifactCount > 3) artifactCount = 3;

                inCooldown = true;
                cooldownStart = getRuntime() * 1000;
            }

            if (!intakeRunning) {
                inCooldown = false;
            }

            lastIntakePos = intakePos;
            lastIntakeVel = intakeVel;

            // ================= LIGHT LOGIC =================
            if (locked) {

                vision.setPosition(PURPLE);
                vision1.setPosition(PURPLE);

            } else {

                if (artifactCount == 1) {
                    vision.setPosition(GREEN);

                } else if (artifactCount == 2) {
                    vision.setPosition(YELLOW);

                } else if (artifactCount == 3) {
                    vision.setPosition(ORANGE);

                } else {
                    vision.setPosition(OFF);
                }

                vision1.setPosition(vision.getPosition());
            }

            // ================= DRIVE =================
            double fl = (y + x + rotation) * speed;
            double bl = (y - x + rotation) * speed;
            double fr = (y - x - rotation) * speed;
            double br = (y + x - rotation) * speed;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl),
                            Math.max(Math.abs(fr), Math.abs(br))));

            if (max > 1) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            robot.frontLeftMotor.setPower(fl);
            robot.backLeftMotor.setPower(bl);
            robot.frontRightMotor.setPower(fr);
            robot.backRightMotor.setPower(br);

            // ================= TELEMETRY =================
            telemetry.addData("Artifacts", artifactCount);
            telemetry.addData("Intake Pos", intakePos);
            telemetry.addData("Delta Pos", deltaPos);
            telemetry.addData("Intake Vel", intakeVel);
            telemetry.addData("Transfer Detected", transferDetected);
            telemetry.addData("Locked", locked);
            telemetry.update();
        }
    }
}