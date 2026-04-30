package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueAutoClose")
public class BlueAutoClose extends OpMode {

    private Follower follower;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime stepTimer = new ElapsedTime();

    private final double TARGET_VELOCITY = 1220;
    private final double VELOCITY_TOLERANCE = 40;

    private DcMotorEx intake, kicker;
    private DcMotorEx launcher, launcher2;
    private Servo wall;

    private final Pose startPose = new Pose(18, 122.5, Math.toRadians(130));
    private final Pose scorePose = new Pose(46, 91, Math.toRadians(125));
    private final Pose scorePose2 = new Pose(52, 112, Math.toRadians(145));

    private final Pose pickup1Pose = new Pose(17, 80, Math.toRadians(175));
    private final Pose pickup2Pose = new Pose(4, 56, Math.toRadians(175));
    private final Pose pickup3Pose = new Pose(4, 34, Math.toRadians(175));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private int step = 0;

    private boolean isLaunching = false;
    private double launchStartTime = 0;
    private final double LAUNCH_TIME = 2;
    private boolean waitingForSpeed = false;

    private void startLaunch() {
        waitingForSpeed = true;
        isLaunching = false;
    }

    private void updateLaunch() {

        double v1 = launcher.getVelocity();
        double v2 = launcher2.getVelocity();

        boolean atSpeed =
                Math.abs(v1 - TARGET_VELOCITY) < VELOCITY_TOLERANCE &&
                        Math.abs(v2 - TARGET_VELOCITY) < VELOCITY_TOLERANCE;

        if (waitingForSpeed && atSpeed) {
            waitingForSpeed = false;
            isLaunching = true;
            launchStartTime = timer.seconds();

            kicker.setPower(1);
        }

        if (isLaunching && timer.seconds() - launchStartTime > LAUNCH_TIME) {
            kicker.setPower(0);
            isLaunching = false;
        }
    }

    private void intakeIn() {
        intake.setPower(0.6);
        kicker.setPower(1);
    }

    private void intakeStop() {
        intake.setPower(0);
        kicker.setPower(0);
    }

    private void wallUp() {
        wall.setPosition(0.11);
    }

    private void wallDown() {
        wall.setPosition(0.32);
    }

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(52.8, 74, 0), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(58.5, 47, 0), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(41, 64, 0), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(58, 16, 0), pickup3Pose))
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
        follower.setMaxPower(1);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        wall = hardwareMap.get(Servo.class, "wall");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        kicker.setDirection(DcMotorSimple.Direction.REVERSE);

        wallUp();

        buildPaths();
        follower.setStartingPose(startPose);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(50, 0, 0, 15.4);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    @Override
    public void start() {
        step = 0;
        wallUp();
        follower.followPath(scorePreload);
        stepTimer.reset();
    }

    @Override
    public void loop() {

        updateLaunch();

        launcher.setVelocity(TARGET_VELOCITY);
        launcher2.setVelocity(TARGET_VELOCITY);

        follower.update();

        switch (step) {

            case 0:
                if (!follower.isBusy()) {
                    stepTimer.reset();
                    step++;
                }
                break;

            case 1:
                if (stepTimer.seconds() > 2 && !isLaunching) {
                    startLaunch();
                }

                if (stepTimer.seconds() > 3) {
                    stepTimer.reset();
                    wallDown();
                    follower.setMaxPower(0.9);
                    follower.followPath(grabPickup1);
                    step++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
                    step++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (!isLaunching) startLaunch();
                    stepTimer.reset();
                    step++;
                }
                break;

            case 4:
                if (stepTimer.seconds() > 1.24) {
                    intakeIn();
                    wallDown();
                    follower.setMaxPower(0.9);
                    follower.followPath(grabPickup2);
                    step++;
                    stepTimer.reset();
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2);
                    step++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (!isLaunching) startLaunch();
                    stepTimer.reset();
                    step++;
                }
                break;

            case 7:
                if (stepTimer.seconds() > 1.25) {
                    intakeIn();
                    wallDown();
                    follower.setMaxPower(0.9);
                    follower.followPath(grabPickup3);
                    step++;
                    stepTimer.reset();
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

            case 9:
                if (!follower.isBusy()) {
                    if (!isLaunching) startLaunch();
                    stepTimer.reset();
                    step++;
                }
                break;

            case 10:
                if (stepTimer.seconds() > 2) {
                    requestOpModeStop();
                }
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Launching", isLaunching);
        telemetry.addData("L1 Vel", launcher.getVelocity());
        telemetry.addData("L2 Vel", launcher2.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setVelocity(0);
        launcher2.setVelocity(0);
        intakeStop();
    }
}