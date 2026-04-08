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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    // === INTAKE ===
    private DcMotorEx intake, kicker;

    // === WALL SERVO ===
    private Servo wall;

    private final Pose startPose = new Pose(18, 122.5, Math.toRadians(135));
    private final Pose scorePose = new Pose(46, 91, Math.toRadians(130));
    private final Pose scorePose2 = new Pose(52, 112, Math.toRadians(145));

    private final Pose pickup1Pose = new Pose(19.3, 82.8, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(10, 58, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(9, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private int step = 0;

    // === INTAKE FUNCTIONS ===
    private void intakeIn() {
        intake.setPower(.7);
        kicker.setPower(1);
    }

    private void intakeInFast() {
        intake.setPower(.9);
        kicker.setPower(1);
    }

    private void intakeOut() {
        intake.setPower(-1);
        kicker.setPower(-1);
    }

    private void intakeStop() {
        intake.setPower(0);
        kicker.setPower(0);
    }

    // === WALL FUNCTIONS ===
    private void wallUp() {
        wall.setPosition(0.15);
    }

    private void wallDown() {
        wall.setPosition(0.32);
    }

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(47, 74, 0), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(52, 46, 0), pickup2Pose))
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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        kicker.setDirection(DcMotorSimple.Direction.REVERSE);

        wallUp(); // start in scoring position

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        step = 0;
        wallUp(); // ensure up for preload
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

            case 1:
                if (timer.seconds() > .7) {
                    wallDown(); // 🔥 DROP WALL FOR PICKUP
                    intakeInFast();
                    follower.setMaxPower(.8);
                    follower.followPath(grabPickup1);
                    step++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeStop();
                    wallUp(); // 🔥 LIFT WALL TO SCORE
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
                    step++;
                }
                break;

            case 3:
                if (!follower.isBusy() && timer.seconds() > .7) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;

            case 4:
                if (timer.seconds() > .5) {
                    intakeStop();
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.8);
                    follower.followPath(grabPickup2);
                    step++;
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
                if (!follower.isBusy() && timer.seconds() > .7) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;

            case 7:
                if (timer.seconds() > 0.7) {
                    intakeStop();
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.8);
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

            case 9:
                if (!follower.isBusy()) {
                    intakeOut();
                    timer.reset();
                    step++;
                }
                break;
        }

        telemetry.addData("Step", step);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeStop();
    }
}