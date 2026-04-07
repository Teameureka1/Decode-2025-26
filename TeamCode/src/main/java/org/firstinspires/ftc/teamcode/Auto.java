package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private final Pose startPose = new Pose(18, 122.5, Math.toRadians(135));
    private final Pose scorePose = new Pose(46, 91, Math.toRadians(129.5));
    private final Pose scorePose2 = new Pose(52, 112, Math.toRadians(144));

    private final Pose pickup1Pose = new Pose(23, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(23, 57, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(22, 34, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private int step = 0;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(55, 77, 0), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(52.4, 49.7, 0), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(57, 20, 0), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        step = 0;
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {

        follower.update();

        switch (step) {

            case 0: // finished preload score
                if (!follower.isBusy()) {
                    timer.reset();
                    step++;
                }
                break;

            case 1: // WAIT at preload score
                if (timer.seconds() > 0.5) {
                    follower.followPath(grabPickup1);
                    step++;
                }
                break;

            case 2: // go pickup 1
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    step++;
                }
                break;

            case 3: // WAIT at score 1
                if (!follower.isBusy()) {
                    timer.reset();
                    step++;
                }
                break;

            case 4:
                if (timer.seconds() > 0.5) {
                    follower.followPath(grabPickup2);
                    step++;
                }
                break;

            case 5: // go pickup 2
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    step++;
                }
                break;

            case 6: // WAIT at score 2
                if (!follower.isBusy()) {
                    timer.reset();
                    step++;
                }
                break;

            case 7:
                if (timer.seconds() > 0.5) {
                    follower.followPath(grabPickup3);
                    step++;
                }
                break;

            case 8: // go pickup 3
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3);
                    step++;
                }
                break;

            case 9: // WAIT at final score
                if (!follower.isBusy()) {
                    timer.reset();
                    step++;
                }
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}