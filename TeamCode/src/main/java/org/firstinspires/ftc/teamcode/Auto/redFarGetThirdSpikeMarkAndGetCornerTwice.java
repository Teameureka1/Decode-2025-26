package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "redFarGetThirdSpikeMarkAndGetCornerTwice")
public class redFarGetThirdSpikeMarkAndGetCornerTwice extends OpMode {

    Config robot;


    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private double targetVelocity = 1560;

    private Path scorePreload;
    private PathChain setUp3, grabPickup3, scorePickup3, setUpNearWall, grabNearWall, getBack, grabAgain, scoreNearWall, setUpNearWall2, grabNearWall2, setUpNearWall3, grabAwayFromWallTowardsGoal, scoreLast, Park;
    private int step = 0;

    // === INTAKE FUNCTIONS ===
    private void intakeIn() {
        robot.intake.setVelocity(1260);
        robot.kicker.setPower(1);
    }

    private void intakeStop() {
        robot.intake.setVelocity(0);
        robot.kicker.setPower(0);
    }

    // === WALL FUNCTIONS ===
    private void wallUp() {
        robot.wall.setPosition(0.15);
    }

    private void wallDown() {
        robot.wall.setPosition(0.32);
    }

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(robot.redStartFar, robot.redFarScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.redStartFar.getHeading(), robot.redFarScorePose.getHeading());

        setUp3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redSetup3Pose))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redSetup3Pose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redSetup3Pose, robot.redPickup3Pose))
                .setLinearHeadingInterpolation(robot.redSetup3Pose.getHeading(), robot.redPickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup3Pose, robot.redFarScorePose))
                .setLinearHeadingInterpolation(robot.redPickup3Pose.getHeading(), robot.redFarScorePose.getHeading())
                .build();
        setUpNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarScorePose, robot.redFarGrabFromHumanPlayerZoneOffTheWallSetup))
                .setLinearHeadingInterpolation(robot.redFarScorePose.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallSetup.getHeading())
                .build();
        grabNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallSetup, robot.redFarGrabFromHumanPlayerZoneOffTheWall))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallSetup.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWall.getHeading())
                .build();
        getBack = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWall, robot.redFarGrabFromHumanPlayerZoneOffTheWallGetBack))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWall.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallGetBack.getHeading())
                .build();
        grabAgain = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallGetBack, robot.redFarGrabFromHumanPlayerZoneOffTheWall))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallGetBack.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWall.getHeading())
                .build();
        scoreNearWall = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWall, robot.redFarScorePose))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWall.getHeading(), robot.redFarScorePose.getHeading())
                .build();
        setUpNearWall2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarScorePose, robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup))
                .setLinearHeadingInterpolation(robot.redFarScorePose.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup.getHeading())
                .build();
        grabNearWall2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup, robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer.getHeading())
                .build();
        setUpNearWall3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer, robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup.getHeading())
                .build();
        grabAwayFromWallTowardsGoal = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup, robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup.getHeading(), robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore.getHeading())
                .build();
        scoreLast = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore, robot.redFarScorePose))
                .setLinearHeadingInterpolation(robot.redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore.getHeading(), robot.redFarScorePose.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(robot.redFarScorePose, robot.redFarPark))
                .setLinearHeadingInterpolation(robot.redFarScorePose.getHeading(), robot.redFarPark.getHeading())
                .build();

    }

    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        wallUp();
        buildPaths();
        follower.setStartingPose(robot.redStartFar);
    }

    @Override
    public void start() {
        step = 0;
        robot.launcher.setVelocity(targetVelocity);
        robot.launcher2.setVelocity(targetVelocity);
        wallUp();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {
        follower.update();

        switch (step) {

            case 0:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1500) {
                    timer.reset();
                    intakeIn();
                    step++;
                }
                break;

            case 1:
                if (timer.seconds() > 1.75) {
                    timer.reset();
                    follower.setMaxPower(1);
                    intakeStop();
                    follower.followPath(setUp3);
                    step++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.8);
                    follower.followPath(grabPickup3);
                    timer.reset();
                    step++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3);
                    timer.reset();
                    step++;
                }
                break;

            case 4:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1500) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;

            case 5:
                if (timer.seconds() > 1.75) {
                    wallDown();
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall);
                    timer.reset();
                    step++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intakeIn();
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
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scoreNearWall);
                    timer.reset();
                    step++;
                }
                break;

            case 10:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1500) {
                    if (timer.seconds() > .5) {
                        wallUp();
                        intakeIn();
                        timer.reset();
                        step++;
                    }
                    break;

                }
                break;

            case 11:
                if (timer.seconds() > 1.75) {
                    wallDown();
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall2);
                    timer.reset();
                    step++;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(1);
                    follower.followPath(grabNearWall2);
                    timer.reset();
                    step++;
                }
                break;

            case 13:
                if (!follower.isBusy() && timer.seconds() > 1.5) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(setUpNearWall3);
                    timer.reset();
                    step++;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(1);
                    follower.followPath(grabAwayFromWallTowardsGoal);
                    timer.reset();
                    step++;
                }
                break;

            case 15:
                if (!follower.isBusy() && timer.seconds() > 1.5) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(scoreLast);
                    timer.reset();
                    step++;
                }
                break;

            case 16:
                if (timer.seconds() > 1.5 && !follower.isBusy() && robot.launcher2.getVelocity() > 1500) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;

            case 17:
                if (timer.seconds() > 1.75) {
                    wallDown();
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(Park);
                    timer.reset();
                    step++;
                }
                break;

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
