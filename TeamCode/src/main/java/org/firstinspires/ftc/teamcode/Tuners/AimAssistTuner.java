package org.firstinspires.ftc.teamcode.Tuners;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class AimAssistTuner extends OpMode {

    private Config robot;

    Follower follower;
    ElapsedTime timer = new ElapsedTime();
    double lastCalcTime = 0;
    boolean aimAssist = false;
    // AIM ASSIST

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;

    double intergralSum = 0.0;
    double lastError = 0.0;
    long lastTime = 0;

    double maxOutput = 1;
    double minOutput = -1;

    double[] stepSizes = {10.0, 1.0, 0.1, .001, .001};

    int stepIndex = 1;


    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(robot.blueStartFar);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();



        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kI += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kI -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadLeftWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad2.dpadRightWasPressed()) {
            kD -= stepSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()) {
            kF += stepSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()) {
            kF -= stepSizes[stepIndex];
        }


        // Driver Controls
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);
        double y = -gamepad1.left_stick_y * speed;
        double x = -gamepad1.left_stick_x * speed;
        double rotation = -gamepad1.right_stick_x * speed;


        if (gamepad1.aWasPressed()) {
            aimAssist = true;
        }

        if (Math.abs(gamepad1.right_stick_x) >= 0.1) {
            aimAssist = false;
        }
        double delay = 20;
        if (aimAssist) {
            if (timer.milliseconds() > lastCalcTime + delay) {
                double headingCalc = robot.blueGetGoalHeading(follower.getPose());
                double error = headingCalc - follower.getHeading();
                error = robot.angleWrap(error);
                telemetry.addData("Error", error);
                long now = System.nanoTime();
                double deltaTime = (now - lastTime) / 1e9;
                lastTime = now;

                // PID Terms
                // intergralSum = I how much error I accumulate in time
                // derivative = D check pass data and use it to correct future problems
                intergralSum += error * deltaTime;
                double derivative = deltaTime > 0 ? (error - lastError) : 0.0;
                lastError = error;

                double pTerm = kP * error;
                double iTerm = kI * intergralSum;
                double dTerm = kD * derivative;

                // FeedForward
                double fTerm = kF * Math.signum(error);

                double output = pTerm + iTerm + dTerm + fTerm;

                if (output > maxOutput) output = maxOutput;
                if (output < minOutput) output = minOutput;

                rotation = output;
                telemetry.addData("error", error);

            }
        }
        // ================= DRIVE =================


        follower.setTeleOpDrive(y, x, rotation, true);


        telemetry.addLine("------------------------------------------------");

        telemetry.addData("kP dpad sideways", kP);
        telemetry.addData("kI  dpad up", kI);
        telemetry.addData("kD  dpad sideway", kD);
        telemetry.addData("kF  dpad up", kF);

        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
