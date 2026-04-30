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

@Autonomous(name = "BlueGate")
public class BlueGate extends OpMode {

    Config robot;


    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private Path scorePreload;
    private PathChain setUp2, grabPickup2, toGate, scorePickup2; // grabPickup1, scorePickup1, grabPickup3, scorePickup3;

    private int step = 0;

    // === INTAKE FUNCTIONS ===
    private void intakeIn() {
        robot.intake.setPower(.65);
        robot.kicker.setPower(1);
    }

    private void intakeInFast() {
        robot.intake.setPower(.75);
        robot.kicker.setPower(1);
    }

    private void intakeOut() {
        robot.intake.setPower(-1);
        robot.kicker.setPower(-1);
    }

    private void intakeStop() {
        robot.intake.setPower(0);
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

        scorePreload = new Path(new BezierLine(robot.blueStartPose, robot.blueScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.blueStartPose.getHeading(), robot.blueScorePose.getHeading());

        setUp2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.blueScorePose, robot.blueSetup2Pose))
                .setLinearHeadingInterpolation(robot.blueScorePose.getHeading(), robot.blueSetup2Pose.getHeading())
                .build();

        toGate = follower.pathBuilder()
                .addPath(new BezierCurve(robot.bluePickup2Pose, new Pose(50, 50, 270), robot.blueGate))
                .setLinearHeadingInterpolation(robot.bluePickup2Pose.getHeading(), robot.blueGate.getHeading())
                .build();


            /*scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(52.8, 74, 0), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
*/
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(robot.bluePickup2Pose, new Pose(41, 64, 0), robot.blueScorePose))
                .setLinearHeadingInterpolation(robot.bluePickup2Pose.getHeading(), robot.blueScorePose.getHeading())
                .build();
/*
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(58, 16, 0), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();

*/
    }

    @Override
    public void init() {
        robot = new Config(this);
        robot.init();

        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);

        wallUp();

        buildPaths();
        follower.setStartingPose(robot.blueStartPose);
    }

    @Override
    public void start() {
        step = 0;
        wallUp();
        follower.followPath(scorePreload);
    }

    @Override
    public void loop() {

        follower.update();

        switch (step) {

            case 0:
                if (!follower.isBusy()) {
                    timer.reset();
                    step++;
                }
                break;
/*
            // === PRELOAD SPIT (2 sec) ===
             case 1:
                intakeOut();
                if (timer.seconds() > 1) {
                    intakeStop();
                    timer.reset();
                    wallDown();
                    intakeInFast();
                    follower.setMaxPower(.95);
                    follower.followPath(grabPickup1);
                    step++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
                    step++;
                }
                break;

            // === SPIT 1 (2 sec) ===
            case 1:
                if (!follower.isBusy()) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;
*/
            case 1:
                if (timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.95);
                    follower.followPath(grabPickup2);
                    step++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(toGate);
                    step++;
                }
            case 3:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    step++;
                }

/*
            case 4:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    step++;
                }
                break;

            // === SPIT 2 (2 sec) ===
            case 6:
                if (!follower.isBusy()) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;

            case 7:
                if (timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.95);
                    follower.followPath(grabPickup3);
                    step++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3);
                    step++;
                }
                break;

            // === FINAL SPIT (2 sec) ===
            case 9:
                if (!follower.isBusy()) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;
*/
            case 4:
                if (!follower.isBusy()) {
                    intakeStop();
                    step++;
                }
                break;
        }




        telemetry.addData("Step", step);
        telemetry.update();
    }
}
