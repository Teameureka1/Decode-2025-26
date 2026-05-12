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
import org.firstinspires.ftc.teamcode.Configuration.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueCloseOpenGateTwiceAndGetThirdSpike")
public class blueCloseOpenGateTwiceAndGetThirdSpike extends OpMode {

    Config robot;


    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private Path scorePreload;
    private PathChain setUp2, grabPickup2, gateSetup, toGate2, scorePickup2, setUp1, grabPickup1, gateSetup2, toGate1, scorePickup1, setUp3, grabPickup3, scorePickup3;

    private int step = 0;

    // === INTAKE FUNCTIONS ===
    private void intakeIn() {
        robot.intake.setVelocity(1200);
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

        scorePreload = new Path(new BezierLine(robot.blueStartClose, robot.blueScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.blueStartClose.getHeading(), robot.blueScorePose.getHeading());

        setUp2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueScorePose, robot.blueSetup2Pose))
                .setLinearHeadingInterpolation(robot.blueScorePose.getHeading(), robot.blueSetup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueSetup2Pose, robot.bluePickup2Pose))
                .setLinearHeadingInterpolation(robot.blueSetup2Pose.getHeading(), robot.bluePickup2Pose.getHeading())
                .build();

        gateSetup = follower.pathBuilder()
                .addPath(new BezierLine(robot.bluePickup2Pose, robot.blueGateSetupPose))
                .setLinearHeadingInterpolation(robot.bluePickup2Pose.getHeading(), robot.blueGateSetupPose.getHeading())
                .build();

        toGate2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.bluePickup2Pose, robot.blueGate))
                .setLinearHeadingInterpolation(robot.bluePickup2Pose.getHeading(), robot.blueGate.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(robot.blueGate, new Pose(60,40,0), robot.blueScorePose))
                .setLinearHeadingInterpolation(robot.blueGate.getHeading(), robot.blueScorePose.getHeading())
                .build();
        setUp1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueScorePose, robot.blueSetup1Pose))
                .setLinearHeadingInterpolation(robot.blueScorePose.getHeading(), robot.blueSetup1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueSetup1Pose, robot.bluePickup1Pose))
                .setLinearHeadingInterpolation(robot.blueSetup1Pose.getHeading(), robot.bluePickup1Pose.getHeading())
                .build();
        gateSetup2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.bluePickup1Pose, robot.blueGateFacingGoalSetup))
                .setLinearHeadingInterpolation(robot.bluePickup1Pose.getHeading(), robot.blueGateFacingGoalSetup.getHeading())
                .build();
        toGate1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueGateFacingGoalSetup, robot.blueGateFacingGoal))
                .setLinearHeadingInterpolation(robot.blueGateFacingGoalSetup.getHeading(), robot.blueGateFacingGoal.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueGateFacingGoal, robot.blueScorePose))
                .setLinearHeadingInterpolation(robot.blueGateFacingGoal.getHeading(), robot.blueScorePose.getHeading())
                .build();
        setUp3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueScorePose, robot.blueSetup3Pose))
                .setLinearHeadingInterpolation(robot.blueScorePose.getHeading(), robot.blueSetup3Pose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueSetup3Pose, robot.bluePickup3Pose))
                .setLinearHeadingInterpolation(robot.blueSetup3Pose.getHeading(), robot.bluePickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.bluePickup3Pose, robot.blueScorePose2))
                .setLinearHeadingInterpolation(robot.bluePickup3Pose.getHeading(), robot.blueScorePose2.getHeading())
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
        follower.setStartingPose(robot.blueStartClose);
    }

    @Override
    public void start() {
        step = 0;
        robot.launcher.setVelocity(1220);
        robot.launcher2.setVelocity(1220);
        wallUp();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {

        follower.update();

        switch (step) {

            case 0:
                if (!follower.isBusy() && robot.launcher2.getVelocity() > 1200) {
                    timer.reset();
                    intakeIn();
                    step++;
                }
                break;

            case 1:
                if (timer.seconds() > 1) {
                    timer.reset();
                    follower.setMaxPower(1);
                    intakeStop();
                    follower.followPath(setUp2);
                    step++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup2);
                    timer.reset();
                    step++;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(gateSetup);
                    timer.reset();
                    step++;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(toGate2);
                    timer.reset();
                    step++;
                }
                break;


            case 5:
                if (!follower.isBusy() && timer.seconds() > 1.5) {
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    timer.reset();
                    step++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;
            case 7:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(setUp1);
                    timer.reset();
                    step++;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup1);
                    timer.reset();
                    step++;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(gateSetup2);
                    timer.reset();
                    step++;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(toGate1);
                    timer.reset();
                    step++;
                }
                break;
            case 11:
                if (!follower.isBusy() && timer.seconds() > 1.5) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
                    timer.reset();
                    step++;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;
            case 13:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(setUp3);
                    timer.reset();
                    step++;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup3);
                    timer.reset();
                    step++;
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3);
                    timer.reset();
                    step++;
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;
            case 17:
                if (!follower.isBusy() && timer.seconds() > 2.5) {
                    requestOpModeStop();
                }

        }

        telemetry.addData("launcher Velocity", robot.launcher.getVelocity());
        telemetry.addData("launcher2 Velocity", robot.launcher2.getVelocity());

        telemetry.addData("Step", step);
        telemetry.addData("F", follower.isBusy());
        telemetry.update();
    }
}
