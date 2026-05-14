package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "AutoDistance")
public class AutoDistance extends OpMode {
    Follower follower;
    double velocity = 0;
    double[] stepSizes = {1000.00, 100.00, 10.0};
    boolean intakeIsOn = false;

    int stepIndex = 1;

    private Config robot;

    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
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

        double throttle = .3 + (gamepad1.right_trigger * .7);
        follower.update();
        follower.setTeleOpDrive(
                (-gamepad1.left_stick_y) * throttle ,
                (-gamepad1.left_stick_x) * throttle,
                (-gamepad1.right_stick_x) * throttle,
                true
        );

        if (gamepad1.aWasPressed()) {
            intakeIsOn = !intakeIsOn;
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }

        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            velocity += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            velocity -= stepSizes[stepIndex];
        }
        robot.launcher.setVelocity(velocity);
        robot.launcher2.setVelocity(velocity);

        double distance = robot.blueGetDistanceFromGoal(follower.getPose());


        telemetry.addData("Distance", distance);
        telemetry.addLine("------------------------------------------------");
        telemetry.addData("Velocity", "%.2f", velocity);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();



    }
}

