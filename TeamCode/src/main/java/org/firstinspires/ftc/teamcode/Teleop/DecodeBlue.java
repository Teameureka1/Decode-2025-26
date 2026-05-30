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

@TeleOp(name = "!Decode Blue")
public class DecodeBlue extends OpMode {

    // Pose for parking
    Pose park;

    // These are variables for my Teleop
    boolean aimAssist = false;
    boolean locked = false;
    boolean intakeIsOn = false;
    double lastCalcTime = 0;


    boolean rumble1Triggered = false;
    boolean rumble2Triggered = false;

    boolean wallClosed = false;



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

    /* Inside the start loop, I have a switch case that acts like a elevator panel, mattering
       what button you press. Instead of buttons, it is autonomous opmodes for us. This allows us to
       save where we end in our autonomous and send it to the start of our teleop. */


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

        if (timer.seconds() > 1) {
            robot.wallClose();
            wallClosed = true;
        }


        if (timer.seconds() > 97) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            rumble1Triggered = true;
        }
        if (timer.seconds() > 113) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            rumble2Triggered = true;
        }

        // Driver Controls
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);
        double y = -gamepad1.left_stick_y * speed;
        double x = -gamepad1.left_stick_x * speed;
        double rotation = -gamepad1.right_stick_x * speed;

        locked = false;

        // ================= LIMELIGHT =================


        /* Left trigger activates Limelight tag tracking — overrides aim assist rotation.
           We are using the Limelight as a backup to the aim assist because the aim assist gets off
           over time, due to the inaccuracy of time and battery drain.
         */


        if (gamepad1.left_trigger > 0.1) {
            aimAssist = false;

            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult fr : tags) {
                    if (fr.getFiducialId() == 20) {
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
        // We use the aim assist to turn to the goal much faster the Limelight and to make it
        // easier as well on the drive team.
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

        /* This is used for holding a certain position on the field whether it is at the correct
           spot or not. We are using this to hold our position while parking, because this allows
           us to correct back to where we were and still get a full park and be fairly accurate. */


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

        // If B is press then it will activate the holding point
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

            follower.setTeleOpDrive(correctionX, correctionY,
                    correctionHeading, true);
        }


        // ================= LAUNCHER =================
        // This is all that we need for launching, because all the logic and movement is inside
        // the configuration file.


        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.blueLaunchThreeUpdater(follower);

        // ================ INTAKE ====================
        /* If I want the intake on press B, but if you want the intake to stop then press B again.
           This is called a toggle, where if you press something again it will toggle on and off,
           kinda like a light switch.  */


        if (gamepad2.bWasPressed()) {
            intakeIsOn = !intakeIsOn;
            if (intakeIsOn) {
                robot.wallClose();
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }
        }

        if (gamepad2.xWasPressed()) {
            robot.intakeOut();
        }

        // ================ WALL =============================
        // Just in case the wall doesn't close in auto or if the wall gets stuck and jams the
        // artifacts inside the intake while launching.

        if (gamepad2.dpadUpWasPressed()) {
            robot.wallOpen();
        }

        if (gamepad2.dpadDownWasPressed()) {
            robot.wallClose();
        }

        // ================= COLOR SENSORS =================
        // This is where I say hey, is there three artifacts or not?

        if (robot.intakeSensor.alpha() > robot.COLORIntake_THRESHOLD &&
                robot.transferSensor.alpha() > robot.COLORTransfer_THRESHOLD) {
            robot.intakeFull = true;
        } else {
            robot.intakeFull = false;
        }


        // ================= LIGHT SYSTEM =================
        // This is where I set it to a certain color based on a color chart on the GoBilda website

        if (locked) {
            robot.vision1.setPosition(robot.GREEN); // Locked onto tag 20
        } else if (aimAssist) {
            robot.vision1.setPosition(robot.GREEN);  // PIDF aim assist active
        } else if (robot.intakeFull) {
            robot.vision.setPosition(robot.ORANGE);  // Intake has three artifacts

        } else {
            robot.vision.setPosition(robot.OFF);
            robot.vision1.setPosition(robot.OFF);
        }

        // ================= TELEMETRY =================
        telemetry.addData("Aim Assist", aimAssist);
        telemetry.addData("Intake Sensor", robot.intakeSensor.alpha());
        telemetry.addData("Transfer Sensor", robot.transferSensor.alpha());
        telemetry.addData("Launch Velocity:", robot.launcher.getVelocity());
        telemetry.addData("Launch2 Velocity:", robot.launcher2.getVelocity());
        telemetry.addData("Aim Mode", gamepad1.a ? "POSE" : "MANUAL");
        telemetry.addData("Auto Ran", Config.lastAutoRun ==
                0 ? "None" : "Auto " + Config.lastAutoRun);
        telemetry.addData("Starting Pose", Config.savedPose !=
                null ? Config.savedPose : "Default");
        telemetry.addLine("Current Pose Coordinates");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Parking", park);
        telemetry.update();
    }
}