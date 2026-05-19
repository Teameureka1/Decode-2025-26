package org.firstinspires.ftc.teamcode.Teleop;



import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Aim Assist")
public class AimAssist extends OpMode {

    boolean aimAssist = false;
    Follower follower;
    private Config robot;
    boolean intakeIsOn = false;
    ElapsedTime timer = new ElapsedTime();
    double lastCalcTime = 0;

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
        double rotation = gamepad1.right_stick_x;
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);

        // ================= ROTATION SOURCE =================

        if (gamepad1.aWasPressed()) {
            aimAssist = true;
        }

        if (Math.abs(gamepad1.right_stick_x) >= 0.1) {
            aimAssist = false;
        }

        if (aimAssist) {

            double delay = 20;
            if (timer.milliseconds() > lastCalcTime + delay) {
                double headingCalc = robot.blueGetGoalHeading(follower.getPose());
                double error = follower.getHeading() - headingCalc;
                telemetry.addData("Error", error);
               // rotation = autocalc


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