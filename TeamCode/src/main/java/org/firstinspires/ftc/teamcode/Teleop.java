package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private Config robot;
    private Follower follower;
    private boolean autoMovement;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
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
        follower.setTeleOpDrive(
                gamepad1.left_stick_y * throttle,
                gamepad1.left_stick_x * throttle,
                gamepad1.right_stick_x * throttle,
                true
        );
        if (gamepad1.aWasPressed()) {
            if (autoMovement == false) {

                autoMovement = true;

            } else {
                autoMovement = false;
            }
        }


    }
}
