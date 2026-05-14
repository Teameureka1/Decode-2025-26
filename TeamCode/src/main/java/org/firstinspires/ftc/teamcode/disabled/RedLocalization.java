package org.firstinspires.ftc.teamcode.disabled;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class RedLocalization extends LinearOpMode {

    @Override
    public void runOpMode() {
        final Pose redStartFar = new Pose(89, 8.39, Math.toRadians(90));

        Follower follower;
        follower = Constants.createFollower(hardwareMap);

        waitForStart();

        follower.setStartingPose(redStartFar);
        while (opModeIsActive()) {
            follower.update();
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double z = follower.getPose().getHeading();
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", Math.toDegrees(z));
            telemetry.update();


        }
    }


}
