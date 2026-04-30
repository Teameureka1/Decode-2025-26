package org.firstinspires.ftc.teamcode.disabled;

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

@Autonomous(name = "RedAuto")
public class RedAuto extends OpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();

    private DcMotorEx intake, kicker;
    private Servo wall;

    // === MIRRORED POSES (CENTER = 72) ===
    private final Pose startPose = new Pose(144 - 18, 122.5, Math.PI - Math.toRadians(135));
    private final Pose scorePose = new Pose(144 - 46, 91, Math.PI - Math.toRadians(130));
    private final Pose scorePose2 = new Pose(144 - 52, 112, Math.PI - Math.toRadians(145));

    private final Pose pickup1Pose = new Pose(144 - 18.5, 80, Math.PI - Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(144 - 10, 58, Math.PI - Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(144 - 9, 36, Math.PI - Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private int step = 0;

    private void intakeIn() {
        intake.setPower(.7);
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

    private void wallDown() {
        wall.setPosition(0.15);
    }

    private void wallUp() {
        wall.setPosition(0.32);
    }

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(144 - 47, 78, 0), pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(144 - 52, 46, 0), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(144 - 41, 64, 0), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(144 - 58, 16, 0), pickup3Pose))
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

        wallUp();

        buildPaths();
        follower.setStartingPose(startPose);
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

            case 1:
                if (timer.seconds() > .7) {
                    wallDown();
                    intakeIn();
                    follower.setMaxPower(.8);
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