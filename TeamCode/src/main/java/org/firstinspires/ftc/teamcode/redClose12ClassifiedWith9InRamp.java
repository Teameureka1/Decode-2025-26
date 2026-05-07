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

@Autonomous(name = "redClose12ClassifiedWith9InRamp")
public class redClose12ClassifiedWith9InRamp extends OpMode {

    Config robot;


    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private Path scorePreload;
    private PathChain setUp2, grabPickup2, gateSetup, toGate, scorePickup2, setUp1, grabPickup1, scorePickup1, setUp3, grabPickup3, scorePickup3, endOfAuto;

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

        scorePreload = new Path(new BezierLine(robot.redStartPose, robot.redScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.redStartPose.getHeading(), robot.redScorePose.getHeading());

        setUp2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redSetup2Pose))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redSetup2Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redSetup2Pose, robot.redPickup2Pose))
                .setLinearHeadingInterpolation(robot.redSetup2Pose.getHeading(), robot.redPickup2Pose.getHeading())
                .build();

        gateSetup = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup2Pose, robot.redSetupGate))
                .setLinearHeadingInterpolation(robot.redPickup2Pose.getHeading(), robot.redSetupGate.getHeading())
                .build();

        toGate = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup2Pose, robot.redGateFacingParkingZone))
                .setLinearHeadingInterpolation(robot.redPickup2Pose.getHeading(), robot.redGateFacingParkingZone.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(robot.redGateFacingParkingZone, new Pose(110,40,0), robot.redScorePose))
                .setLinearHeadingInterpolation(robot.redGateFacingParkingZone.getHeading(), robot.redScorePose.getHeading())
                .build();
        setUp1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redSetup1Pose))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redSetup1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redSetup1Pose, robot.redPickup1Pose))
                .setLinearHeadingInterpolation(robot.redSetup1Pose.getHeading(), robot.redPickup1Pose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup1Pose, robot.redScorePose))
                .setLinearHeadingInterpolation(robot.redPickup1Pose.getHeading(), robot.redScorePose.getHeading())
                .build();
        setUp3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redSetup3Pose))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redSetup3Pose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redSetup3Pose, robot.redPickup3Pose))
                .setLinearHeadingInterpolation(robot.redSetup3Pose.getHeading(), robot.redPickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup3Pose, robot.redScorePose))
                .setLinearHeadingInterpolation(robot.redPickup3Pose.getHeading(), robot.redScorePose.getHeading())
                .build();
        endOfAuto = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redAutoEnd))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redAutoEnd.getHeading())
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
        follower.setStartingPose(robot.redStartPose);
    }

    @Override
    public void start() {
        step = 0;
        robot.launcher.setVelocity(1240);
        robot.launcher2.setVelocity(1240);
        wallUp();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {

        follower.update();

        switch (step) {

            case 0:
                if (!follower.isBusy() && robot.launcher2.getVelocity() < 1240 && timer.seconds() > 3.1) {
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
                if (!follower.isBusy() && timer.seconds() > .25) {
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup2);
                    timer.reset();
                    step++;
                }
                break;
            case 3:
                if (!follower.isBusy() && timer.seconds() > .25) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(gateSetup);
                    timer.reset();
                    step++;
                }
                break;
            case 4:
                if (!follower.isBusy() && timer.seconds() > .8) {
                    follower.setMaxPower(1);
                    follower.followPath(toGate);
                    timer.reset();
                    step++;
                }
                break;


            case 5:
                if (!follower.isBusy() && timer.seconds() > 2) {
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    timer.reset();
                    step++;
                }
                break;

            case 6:
                if (!follower.isBusy() && timer.seconds() > 2.5) {
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
                if (!follower.isBusy() && timer.seconds() > .2) {
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup1);
                    timer.reset();
                    step++;
                }
                break;
            case 9:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
                    timer.reset();
                    step++;
                }
                break;
            case 10:
                if (!follower.isBusy() && timer.seconds() > .5) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;
            case 11:
                if (!follower.isBusy() && timer.seconds() > 1.75) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(setUp3);
                    timer.reset();
                    step++;
                }
                break;
            case 12:
                if (!follower.isBusy() && timer.seconds() > 1.25) {
                    intakeIn();
                    follower.setMaxPower(.65);
                    follower.followPath(grabPickup3);
                    timer.reset();
                    step++;
                }
                break;
            case 13:
                if (!follower.isBusy() && timer.seconds() > .5) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3);
                    timer.reset();
                    step++;
                }
                break;
            case 14:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    wallUp();
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;
            case 15:
                if (!follower.isBusy() && timer.seconds() > 2) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(endOfAuto);
                    timer.reset();
                    step++;
                }
            case 16:
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
