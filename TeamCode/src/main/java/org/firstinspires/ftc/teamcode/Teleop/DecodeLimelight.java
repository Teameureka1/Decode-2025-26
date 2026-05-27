package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "!Decode")
public class DecodeLimelight extends OpMode {

    boolean aimAssist = false;
    boolean locked = false;
    Follower follower;
    private Config robot;
    boolean intakeIsOn = false;
    ElapsedTime timer = new ElapsedTime();
    double lastCalcTime = 0;

    // =============== AIM ASSIST ====================
    // =============== PIDF VALUES ===================
    double kP = 1;
    double kI = 0;
    double kD = 0;
    double kF = .03;

    double intergralSum = 0.0;
    double lastError = 0.0;
    long lastTime = 0;

    double maxOutput = 1;
    double minOutput = -1;


    @Override
    public void init() {
        robot = new Config(this);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        robot.init();
    }

    @Override
    public void start() {
        if (Config.savedPose != null) {
            follower.setStartingPose(Config.savedPose);
        } else {
            switch (Config.lastAutoRun) {
                case 1:
                    follower.setStartingPose(robot.blueAutoEnd);
                    break;
                case 2:
                    follower.setStartingPose(robot.blueScorePose2);
                    break;
                case 3:
                    follower.setStartingPose(robot.blueScorePose2);
                    break;
                case 4:
                    follower.setStartingPose(robot.blueFarPark);
                    break;
                default:
                    follower.setStartingPose(robot.blueStartFar);
                    break;
            }
        }
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Driver Controls
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);
        double y = -gamepad1.left_stick_y * speed;
        double x = -gamepad1.left_stick_x * speed;
        double rotation = -gamepad1.right_stick_x * speed;

        locked = false;

        // ================= LIMELIGHT =================
        // Left trigger activates Limelight tag tracking — overrides aim assist rotation
        if (gamepad1.left_trigger > 0.1) {
            aimAssist = false; // Limelight takes priority; disable PIDF aim assist

            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : tags) {
                    if (fr.getFiducialId() == 20) {
                        double tx = -fr.getTargetXDegrees();

                        if (Math.abs(tx) < 3.0) {          // wider deadband
                            rotation = 0;
                            locked = true;
                        } else {
                            rotation = tx * 0.02;           // lower gain
                            rotation = Math.max(-0.55, Math.min(rotation, 0.55));
                            if (Math.abs(rotation) < 0.05) rotation = 0; // min threshold
                        }

                        telemetry.addData("TX", fr.getTargetXDegrees());
                        telemetry.addData("Rotation Output", rotation);
                    }
                }
            }
        }

        // ================= AIM ASSIST =================
        // Only runs if Limelight is not active
        if (!locked && gamepad1.left_trigger <= 0.1) {

            if (gamepad1.aWasPressed()) {
                aimAssist = true;
            }

            if (Math.abs(gamepad1.right_stick_x) >= 0.1) {
                aimAssist = false;
            }

            if (aimAssist) {
                double delay = 20;
                if (timer.milliseconds() > lastCalcTime + delay) {
                    lastCalcTime = timer.milliseconds();
                    double headingCalc = robot.blueGetGoalHeading(follower.getPose());
                    double error = headingCalc - follower.getHeading();
                    error = robot.angleWrap(error);
                    telemetry.addData("Error", error);
                    long now = System.nanoTime();
                    double deltaTime = (now - lastTime) / 1e9;
                    lastTime = now;

                    intergralSum += error * deltaTime;
                    double derivative = deltaTime > 0 ? (error - lastError) : 0.0;
                    lastError = error;

                    double pTerm = kP * error;
                    double iTerm = kI * intergralSum;
                    double dTerm = kD * derivative;
                    double fTerm = kF * Math.signum(error);

                    double output = pTerm + iTerm + dTerm + fTerm;

                    if (output > maxOutput) output = maxOutput;
                    if (output < minOutput) output = minOutput;

                    rotation = output;
                }
            }
        }

        follower.setTeleOpDrive(y, x, rotation, true);

        // ================= LAUNCHER =================
        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.blueLaunchThreeUpdater(follower);

        // ================ INTAKE ====================
        if (gamepad2.bWasPressed()) {
            intakeIsOn = !intakeIsOn;
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }
        }

        if (gamepad1.xWasPressed()) {
            robot.intakeOut();
        }

        // ================= LIGHT SYSTEM =================
        if (locked) {
            robot.vision1.setPosition(robot.GREEN); // Locked onto tag 20
        } else if (aimAssist) {
            robot.vision1.setPosition(robot.GREEN);  // PIDF aim assist active
        } else {
            robot.vision.setPosition(robot.OFF);
            robot.vision1.setPosition(robot.OFF);
        }

        // ================= TELEMETRY =================
        telemetry.addData("Aim Assist", aimAssist);
        telemetry.addData("Limelight Active", gamepad1.left_trigger > 0.1);
        telemetry.addData("Locked onto Tag 20", locked);
        telemetry.addData("Intake Sensor", robot.intakeSensor.alpha());
        telemetry.addData("Transfer Sensor", robot.transferSensor.alpha());
        telemetry.addData("Launch Velocity:", robot.launcher.getVelocity());
        telemetry.addData("Launch2 Velocity:", robot.launcher2.getVelocity());
        telemetry.addData("Aim Mode", gamepad1.a ? "POSE" : "MANUAL");
        telemetry.addData("Auto Ran", Config.lastAutoRun == 0 ? "None" : "Auto " + Config.lastAutoRun);
        telemetry.addData("Starting Pose", Config.savedPose != null ? Config.savedPose : "Default");
        telemetry.update();
    }
}