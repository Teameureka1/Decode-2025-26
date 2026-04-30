package org.firstinspires.ftc.teamcode;

import com.bylazar.field.Line;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Configuration.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Localization")
public class Localization extends LinearOpMode {

    @Override
    public void runOpMode() {
        final Pose blueStartFar = new Pose(55, 8.39, Math.toRadians(90));
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
            // telemetry.addData("Pose", follower.getPose());
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", Math.toDegrees(z));
            telemetry.update();


        }
    }


}
