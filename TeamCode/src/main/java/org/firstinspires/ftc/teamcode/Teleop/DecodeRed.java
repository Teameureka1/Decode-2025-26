package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "!Decode Red")
public class DecodeRed extends OpMode {

    // Pose for parking
    Pose park;

    // These are variables for my Teleop
    boolean aimAssist = false;
    boolean locked = false;
    boolean intakeIsOn = false;
    double lastCalcTime = 0;


    // More important variables
    Follower follower;
    private Config robot;
    ElapsedTime timer = new ElapsedTime();

    // =============== AIM ASSIST ====================
    // =============== PIDF VALUES ===================
    double kP = 1;
    double kI = 0;
    double kD = 0;
    double kF = .03;

    double holdKP = .12;
    double holdKP2 = .65;
    double holdX = 0;
    double holdY = 0;

    double correctionX = 0;
    double correctionY = 0;
    double correctionHeading = 0;

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
                    follower.setStartingPose(robot.redAutoEnd);
                    break;
                case 2:
                    follower.setStartingPose(robot.redScorePose2);
                    break;
                case 3:
                    follower.setStartingPose(robot.redScorePose2);
                    break;
                case 4:
                    follower.setStartingPose(robot.redFarPark);
                    break;
                default:
                    follower.setStartingPose(robot.redStartFar);
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
                    if (fr.getFiducialId() == 24) {
                        double tx = -fr.getTargetXDegrees();

                        if (Math.abs(tx) < 3.0) {
                            rotation = 0;
                            locked = true;
                        } else {
                            rotation = tx * 0.02;
                            rotation = Math.max(-0.55, Math.min(rotation, 0.55));
                            if (Math.abs(rotation) < 0.05) rotation = 0;
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
                    double headingCalc = robot.redGetGoalHeading(follower.getPose());
                    double error = headingCalc - follower.getHeading();
                    error = Config.angleWrap(error);
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

        // ============== HOLD POSITION ===============


        // Cancel hold when sticks are moved
        if (park != null && (Math.abs(gamepad1.left_stick_x) > 0.15
                || Math.abs(gamepad1.left_stick_y) > 0.15
                || Math.abs(gamepad1.right_stick_x) > 0.15)) {
            park = null;
        }

        // Save position on B press
        if (gamepad1.bWasPressed()) {
            Pose current = follower.getPose();
            holdX = current.getX();
            holdY = current.getY();
            park = current;
        }

        if (park != null) {
            Pose current = follower.getPose();
            double errorX = holdX - current.getX();
            double errorY = holdY - current.getY();
            double heading = follower.getHeading();

            correctionX = (errorX * Math.cos(heading) + errorY * Math.sin(heading)) * holdKP;
            correctionY = (-errorX * Math.sin(heading) + errorY * Math.cos(heading)) * holdKP;

            double headingError = Config.angleWrap(park.getHeading() - follower.getHeading());
            correctionHeading = Math.max(-1, Math.min(headingError * holdKP2, 1));

            correctionX = Math.max(-1, Math.min(correctionX, 1));
            correctionY = Math.max(-1, Math.min(correctionY, 1));

            follower.setTeleOpDrive(correctionX, correctionY, correctionHeading, true);
        }



        // ================= LAUNCHER =================
        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.redLaunchThreeUpdater(follower);

        // ================ INTAKE ====================
        // If I want the intake on I press B, but if I want the intake to stop then I press B again.
        // This is called a toggle, where if you press something again it will toggle on and off,
        // kinda like the ignition to a car that does not use a key to start.
        if (gamepad2.bWasPressed()) {
            intakeIsOn = !intakeIsOn;
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }
        }
        if (gamepad1.yWasPressed()) {
            intakeIsOn = !intakeIsOn;
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }
        }

        if (gamepad2.xWasPressed()) {
            robot.intakeOut();
        }

        // ================ WALL =============================
        // Just in case the wall doesn't close in auto

        if (gamepad2.aWasPressed()) {
            robot.wallClose();
        }

        // ================= COLOR SENSORS =================
        if (robot.intakeSensor.alpha() > robot.COLORIntake_THRESHOLD && robot.transferSensor.alpha() > robot.COLORTransfer_THRESHOLD) {
            robot.intakeFull = true;
        } else {
            robot.intakeFull = false;
        }


        // ================= LIGHT SYSTEM =================
        if (locked) {
            robot.vision1.setPosition(robot.GREEN); // Locked onto tag 20
        } else if (aimAssist) {
            robot.vision1.setPosition(robot.GREEN);  // PIDF aim assist active
        } else if (robot.intakeFull) {
            robot.vision.setPosition(robot.ORANGE);

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
        telemetry.addData("Starting Pose Coordinates", follower.getPose());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Parking", park);
        telemetry.update();
    }
}