package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private Follower follower;
    private boolean autoMovement = false;
    private Pose park;

    @Override
    public void init() {
   //     follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);

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
        if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.right_stick_x) > .1 && autoMovement) {

            autoMovement = false;
            follower.breakFollowing();
            follower.startTeleOpDrive();
        }


        if (!autoMovement){
            follower.setTeleOpDrive(
                    (-gamepad1.left_stick_y) * (throttle + .3),
                    (-gamepad1.left_stick_x) * (throttle + .3),
                    (-gamepad1.right_stick_x) * (throttle + .3),
                    true
            );
        }



        telemetry.addData("Auto Movement", autoMovement);
    }
}
