package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueFarGetThirdSpikeMarkAndGetCornerTwice")
public class blueFarGetThirdSpikeMarkAndGetCornerTwice extends OpMode {

    Config robot;


    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity = 1500;

    private Path scorePreload;
    private PathChain setUp3, grabPickup3, scorePickup3, setUpNearWall, grabNearWall, getBack, grabAgain, scoreNearWall, setUpNearWall2, grabNearWall2, setUpNearWall3, grabAwayFromWallTowardsGoal, scoreLast, Park;
    private int step = 0;


    public void buildPaths() {

        scorePreload = new Path(new BezierLine(robot.blueStartFar, robot.blueFarScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.blueStartFar.getHeading(), robot.blueFarScorePose.getHeading());

        setUp3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueScorePose, robot.blueSetup3Pose))
                .setLinearHeadingInterpolation(robot.blueScorePose.getHeading(), robot.blueSetup3Pose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueSetup3Pose, robot.bluePickup3Pose))
                .setLinearHeadingInterpolation(robot.blueSetup3Pose.getHeading(), robot.bluePickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.bluePickup3Pose, robot.blueFarScorePose))
                .setLinearHeadingInterpolation(robot.bluePickup3Pose.getHeading(), robot.blueFarScorePose.getHeading())
                .build();
        setUpNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarScorePose, robot.blueFarGrabFromHumanPlayerZoneOffTheWallSetup))
                .setLinearHeadingInterpolation(robot.blueFarScorePose.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallSetup.getHeading())
                .build();
        grabNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallSetup, robot.blueFarGrabFromHumanPlayerZoneOffTheWall))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallSetup.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWall.getHeading())
                .build();
        getBack = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWall, robot.blueFarGrabFromHumanPlayerZoneOffTheWallGetBack))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWall.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallGetBack.getHeading())
                .build();
        grabAgain = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallGetBack, robot.blueFarGrabFromHumanPlayerZoneOffTheWall))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallGetBack.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWall.getHeading())
                .build();
        scoreNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWall, robot.blueFarScorePose))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWall.getHeading(), robot.blueFarScorePose.getHeading())
                .build();
        setUpNearWall2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarScorePose, robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup))
                .setLinearHeadingInterpolation(robot.blueFarScorePose.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup.getHeading())
                .build();
        grabNearWall2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup, robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer.getHeading())
                .build();
        setUpNearWall3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer, robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup.getHeading())
                .build();
        grabAwayFromWallTowardsGoal = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup, robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup.getHeading(), robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore.getHeading())
                .build();
        scoreLast = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore, robot.blueFarScorePose))
                .setLinearHeadingInterpolation(robot.blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore.getHeading(), robot.blueFarScorePose.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueFarScorePose, robot.blueFarPark))
                .setLinearHeadingInterpolation(robot.blueFarScorePose.getHeading(), robot.blueFarPark.getHeading())
                .build();

    }

    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        robot.wallOpen();
        buildPaths();
        follower.setStartingPose(robot.blueStartFar);
    }

    @Override
    public void start() {
        step = 0;
        robot.launcher.setVelocity(targetVelocity);
        robot.launcher2.setVelocity(targetVelocity);
        robot.wallOpen();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {
        follower.update();

        switch (step) {

            case 0:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1440) {
                    timer.reset();
                    robot.intakeIn();
                    step++;
                }
                break;

            case 1:
                if (timer.seconds() > 1.5) {
                    timer.reset();
                    follower.setMaxPower(1);
                    robot.intakeStop();
                    follower.followPath(setUp3);
                    step++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    robot.wallClose();
                    robot.intakeIn();
                    follower.setMaxPower(.8);
                    follower.followPath(grabPickup3);
                    timer.reset();
                    step++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3);
                    timer.reset();
                    step++;
                }
                break;

            case 4:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1440) {
                        robot.wallOpen();
                        robot.intakeIn();
                        timer.reset();
                        step++;
                }
                break;

            case 5:
                if (timer.seconds() > 1.5) {
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall);
                    timer.reset();
                    step++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    robot.wallClose();
                    robot.intakeIn();
                    follower.setMaxPower(.8);
                    follower.followPath(grabNearWall);
                    timer.reset();
                    step++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(getBack);
                    timer.reset();
                    step++;
                }
                break;

            case 8:
                if (timer.seconds() > .25) {
                    follower.setMaxPower(1);
                    follower.followPath(grabAgain);
                    timer.reset();
                    step++;
                }
                break;

            case 9:
                if (timer.seconds() > .5) {
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scoreNearWall);
                    timer.reset();
                    step++;
                }
                break;

                case 10:
                    if (timer.seconds() > 2.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1440) {
                        if (timer.seconds() > .5) {
                            robot.wallOpen();
                            robot.intakeIn();
                            timer.reset();
                            step++;
                        }
                        break;

                    }
                    break;

            case 11:
                if (timer.seconds() > 1.75) {
                    robot.wallClose();
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall2);
                    timer.reset();
                    step++;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    robot.intakeIn();
                    follower.setMaxPower(1);
                    follower.followPath(grabNearWall2);
                    timer.reset();
                    step++;
                }
                break;

            case 13:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall3);
                    timer.reset();
                    step++;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    robot.intakeIn();
                    follower.setMaxPower(1);
                    follower.followPath(grabAwayFromWallTowardsGoal);
                    timer.reset();
                    step++;
                }
                break;

            case 15:
                if (!follower.isBusy() && timer.seconds() > .75) {
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scoreLast);
                    timer.reset();
                    step++;
                }
                break;

            case 16:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1440) {
                        robot.wallOpen();
                        robot.intakeIn();
                        timer.reset();
                        step++;
                }
                break;

            case 17:
                if (timer.seconds() > 1.75) {
                    robot.wallClose();
                    robot.intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(Park);
                    timer.reset();
                    step++;
                }
                break;
            case 18:
                if (timer.seconds() > 1.5) {
                    Config.savedPose = follower.getPose();
                    Config.lastAutoRun = 3;
                }

        }

        telemetry.addData("launcher Velocity", robot.launcher.getVelocity());
        telemetry.addData("launcher2 Velocity", robot.launcher2.getVelocity());
        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Step", step);
        telemetry.addData("F", follower.isBusy());
        telemetry.update();
    }
}
