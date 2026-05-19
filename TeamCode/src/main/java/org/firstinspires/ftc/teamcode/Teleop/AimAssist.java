package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Aim Assist")
public class AimAssist extends OpMode {

    boolean poseAimEnabled = false;
    Follower follower;
    private Config robot;
    boolean intakeIsOn = false;

    @Override
    public void init() {

        robot = new Config(this);
        robot.init();
        robot.limelight.pipelineSwitch(8);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(robot.blueStartFar);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
      //  double rotation = gamepad1.right_stick_x * 0.75;
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);


        // ================= ROTATION SOURCE =================
        double rotation = gamepad1.right_stick_x * 0.75;

// toggle pose aim
        if (gamepad1.aWasPressed()) {
            poseAimEnabled = !poseAimEnabled;
        }

// manual cancels aim
        boolean manualDrive =
                Math.abs(gamepad1.left_stick_x) > 0.3 ||
                        Math.abs(gamepad1.left_stick_y) > 0.3 ||
                        Math.abs(gamepad1.right_stick_x) > 0.3;

        if (manualDrive) {
            poseAimEnabled = false;
        }

// ================= POSE AIM =================
        if (poseAimEnabled) {

            double target = robot.blueGetGoalHeading(follower.getPose());
            double current = follower.getPose().getHeading();

            double error = robot.normalizeAngle(target - current);

            double kP = 1.2;

            if (Math.abs(error) < 0.08) {
                rotation = 0;
            } else {
                rotation = error * kP;
                rotation = Math.max(-0.5, Math.min(0.5, rotation));
            }
        }
        // ================= LAUNCHER =================

        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.blueLaunchThreeUpdater(follower);

        if (gamepad2.bWasPressed()) {
            intakeIsOn = !intakeIsOn;
            robot.stopLaunch();
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }

        }

        // ================= COLOR SENSORS =================
        if (robot.intakeSensor.alpha() > robot.COLORIntake_THRESHOLD && robot.transferSensor.alpha() > robot.COLORTransfer_THRESHOLD) {
            robot.intakeFull = true;
        } else {
            robot.intakeFull = false;
        }

        // ================= LIGHT SYSTEM =================
         if (robot.intakeFull) {

            robot.vision.setPosition(robot.ORANGE);

        } else {

            robot.vision.setPosition(robot.OFF);
            robot.vision1.setPosition(robot.OFF);
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
        telemetry.addData("Intake Full", robot.intakeFull);
        telemetry.addData("Intake Sensor", robot.intakeSensor.alpha());
        telemetry.addData("Transfer Sensor", robot.transferSensor.alpha());
        telemetry.addData("Launch Velocity:", robot.launcher.getVelocity());
        telemetry.addData("Launch2 Velocity:", robot.launcher2.getVelocity());
        telemetry.addData("Aim Mode", gamepad1.a ? "POSE" : gamepad1.left_trigger > 0.1 ? "LIMELIGHT" : "MANUAL");
        telemetry.update();
    }
}