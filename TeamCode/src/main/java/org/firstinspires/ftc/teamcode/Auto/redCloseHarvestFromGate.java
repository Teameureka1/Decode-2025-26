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
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name = "redCloseHarvestFromGate")
public class redCloseHarvestFromGate extends OpMode {

    Config robot;


    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();

    private Path scorePreload;
    private PathChain setUp2, grabPickup2, gateSetup, toGate2, scorePickup2, gateGrabSetup, gateGrab, gateHarvest, scoreGrab, setUp1, grabPickup1, toGate1, scorePickup1;

    private int step = 0;

    // === INTAKE FUNCTIONS ===
    private void intakeIn() {
        robot.intake.setVelocity(1300);
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

        scorePreload = new Path(new BezierLine(robot.redStartClose, robot.redScorePose));
        scorePreload.setLinearHeadingInterpolation(robot.redStartClose.getHeading(), robot.redScorePose.getHeading());

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

        toGate2 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup2Pose, robot.redGateFacingParkingZone))
                .setLinearHeadingInterpolation(robot.redPickup2Pose.getHeading(), robot.redGateFacingParkingZone.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(robot.redGateFacingParkingZone, new Pose(110,40,0), robot.redScorePose))
                .setLinearHeadingInterpolation(robot.redGateFacingParkingZone.getHeading(), robot.redScorePose.getHeading())
                .build();
        gateGrabSetup = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redGateHarvestSetup))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redGateHarvestSetup.getHeading())
                .build();
        gateGrab = follower.pathBuilder()
                .addPath(new BezierLine(robot.redGateHarvestSetup, robot.redGateHold))
                .setLinearHeadingInterpolation(robot.redGateHarvestSetup.getHeading(), robot.redGateHold.getHeading())
                .build();
        gateHarvest = follower.pathBuilder()
                .addPath(new BezierLine(robot.redGateHold, robot.redGateHarvest))
                .setLinearHeadingInterpolation(robot.redGateHold.getHeading(), robot.redGateHarvest.getHeading())
                .build();
        scoreGrab = follower.pathBuilder()
                .addPath(new BezierCurve(robot.redGateHarvest, new Pose(103,50, 0), robot.redScorePose))
                .setLinearHeadingInterpolation(robot.redGateHarvest.getHeading(), robot.redScorePose.getHeading())
                .build();

        setUp1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redScorePose, robot.redSetup1Pose))
                .setLinearHeadingInterpolation(robot.redScorePose.getHeading(), robot.redSetup1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redSetup1Pose, robot.redPickup1Pose))
                .setLinearHeadingInterpolation(robot.redSetup1Pose.getHeading(), robot.redPickup1Pose.getHeading())
                .build();
        toGate1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redPickup1Pose, robot.redGateFacingGoal))
                .setLinearHeadingInterpolation(robot.redPickup1Pose.getHeading(), robot.redGateFacingGoal.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(robot.redGateFacingGoal, robot.redScorePose2))
                .setLinearHeadingInterpolation(robot.redGateFacingGoal.getHeading(), robot.redScorePose2.getHeading())
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
        follower.setStartingPose(robot.redStartClose);
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
                if (!follower.isBusy() && robot.launcher2.getVelocity() > 1180 ) {
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
                    follower.setMaxPower(.7);
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
                    follower.followPath(gateGrabSetup);
                    timer.reset();
                    step++;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(1);
                    follower.followPath(gateGrab);
                    timer.reset();
                    step++;
                }
                break;
            case 9:
                if (timer.seconds() > 1) {

                    follower.setMaxPower(1);
                    follower.followPath(gateHarvest);
                    timer.reset();
                    step++;
                }
                break;
            case 10:
                if (!follower.isBusy() && timer.seconds() > 2) {
                    intakeStop();
                    wallUp();
                    follower.setMaxPower(1);
                    follower.followPath(scoreGrab);
                    timer.reset();
                    step++;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeIn();
                    timer.reset();
                    step++;
                }
                break;

            case 12:
                if (!follower.isBusy() && timer.seconds() > 1) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(setUp1);
                    timer.reset();
                    step++;
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeIn();
                    follower.setMaxPower(.7);
                    follower.followPath(grabPickup1);
                    timer.reset();
                    step++;
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intakeStop();
                    follower.setMaxPower(1);
                    follower.followPath(toGate1);
                    timer.reset();
                    step++;
                }
                break;
            case 15:
                if (!follower.isBusy() && timer.seconds() > 1.5) {
                    intakeStop();
                    wallDown();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup1);
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
                if (!follower.isBusy() && timer.seconds() > 3) {
                    wallDown();
                    intakeStop();
                    requestOpModeStop();
                }
                break;
        }

        telemetry.addData("launcher Velocity", robot.launcher.getVelocity());
        telemetry.addData("launcher2 Velocity", robot.launcher2.getVelocity());
        telemetry.addData("Step", step);
        telemetry.addData("F", follower.isBusy());
        telemetry.update();
    }
}