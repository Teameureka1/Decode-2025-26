package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private Config robot;
    private Follower follower;
    private boolean autoMovement = false;
    private Pose park;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        //Runnable pathChain = () -> follower.pathBuilder()
        //        .addPath(new Path(new BezierLine(follower::getPose, follower::getPose)))
        //        .setHeadingInterpolation(HeadingInterpolator.facingPoint(follower..getHeading()))
        //        .build();
    }
    @Override
    public void start() {
        follower.startTeleOpDrive();
    }



    @Override
    public void loop() {
        double throttle = gamepad1.right_trigger;
        follower.update();

        if (gamepad1.bWasPressed() && !autoMovement) {
            autoMovement = true;
            park = follower.getPose();
            follower.holdPoint(park);

        }
        if (Math.abs(gamepad1.left_stick_y)
                > .1 && Math.abs(gamepad1.left_stick_x)
                > .1 && Math.abs(gamepad1.right_stick_x)
                > .1 && autoMovement) {

            autoMovement = false;
            follower.breakFollowing();
            follower.startTeleOpDrive();
         //  follower.followPath(pathChain.getY());

        }


        if (!autoMovement){
            follower.setTeleOpDrive(
                    (-gamepad1.left_stick_y) * (throttle),
                    (-gamepad1.left_stick_x) * (throttle),
                    (-gamepad1.right_stick_x) * (throttle),
                    true
            );
        }



        telemetry.addData("Auto Movement", autoMovement);
    }
}
