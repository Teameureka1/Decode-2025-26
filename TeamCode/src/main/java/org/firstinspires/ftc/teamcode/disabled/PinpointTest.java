package org.firstinspires.ftc.teamcode.disabled;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class PinpointTest extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();


    private final Pose startPose  = new Pose(23, 120, 0);
    private final Pose point1Pose = new Pose(32, 120, 0);
    private final Pose point2Pose = new Pose(32, 111, 0);
    private final Pose point3Pose = new Pose(23, 111, 0);

    private Path path1, path2, path3, path4;

    private int step = 0;

    public void buildPaths() {

        path1 = new Path(new BezierLine(startPose, point1Pose));
        path2 = new Path(new BezierLine(point1Pose, point2Pose));
        path3 = new Path(new BezierLine(point2Pose, point3Pose));
        path4 = new Path(new BezierLine(point3Pose, startPose));
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.4);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        step = 0;
        timer.reset();
        follower.followPath(path1);
    }

    @Override
    public void loop() {

        follower.update();

        switch (step) {

            case 0:
                if (timer.seconds() > 0.8) {
                    follower.followPath(path2);
                    timer.reset();
                    step++;
                }
                break;

            case 1:
                if (timer.seconds() > 0.8) {
                    follower.followPath(path3);
                    timer.reset();
                    step++;
                }
                break;

            case 2:
                if (timer.seconds() > 0.8) {
                    follower.followPath(path4);
                    timer.reset();
                    step++;
                }
                break;

            case 3:
                if (timer.seconds() > 0.8) {
                    step++; // STOP
                }
                break;

            case 4:
                // idle
                break;
        }

        telemetry.addData("Step", step);
        telemetry.update();
    }
}