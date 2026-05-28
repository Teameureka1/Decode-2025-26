package org.firstinspires.ftc.teamcode.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/* In this class basically if you go to another class and say "Config robot" it will take
 all of this class and put it there, but remember to add robot.init inside your init.
 You also you need "robot = new Config(this);" before robot.init.
 */

public class Config {
    private LinearOpMode linearOpMode;
    private OpMode opmode;

    public Config(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public Config(OpMode opmode) {
        this.opmode = opmode;
    }


    public static Pose savedPose = null;
    public static int lastAutoRun = 0;

    public ElapsedTime launchTimer = new ElapsedTime();
    public double lastCalcDistance = 0;
    public double idleVelocity = 1000;

    public void intakeIn() {
        intake.setVelocity(1180);
        kicker.setPower(1);
    }

    public void intakeOut() {
        intake.setVelocity(-1180);
        kicker.setPower(-1);
    }

    public void intakeStop() {
        intake.setVelocity(0);
        kicker.setPower(0);
    }

    public void wallOpen() {
        wall.setPosition(.39);
    }

    public void wallClose() {
        wall.setPosition(.53);
    }

    public void blueLaunchThreeUpdater(Follower follower) {

        if (!launching) {

            return;
        }

        double distance = blueGetDistanceFromGoal(follower.getPose());
        double velocity = calculateLaunchVelocity(distance);
        lastCalcDistance = distance;


        switch (launchSequenceStep) {

            case 1:
                launcher.setVelocity(velocity);
                launcher2.setVelocity(velocity);
                launchSequenceStep++;
                break;
            case 2:
                if (launcher.getVelocity() > velocity - 20) {
                    wallOpen();
                    intakeIn();
                    launchTimer.reset();
                    launchSequenceStep++;
                }
                break;
            case 3:
                if (launchTimer.seconds() > 1.75) {
                    intakeStop();
                    wallClose();
                    launcher.setVelocity(idleVelocity);
                    launcher2.setVelocity(idleVelocity);
                    launchSequenceStep = 1;
                    launching = false;
                }
                break;
        }

    }

    public void redLaunchThreeUpdater(Follower follower) {

        if (!launching) {

            return;
        }


        double velocity = calculateLaunchVelocity(redGetDistanceFromGoal(follower.getPose()));


        switch (launchSequenceStep) {

            case 1:
                launcher.setVelocity(velocity);
                launcher2.setVelocity(velocity);
                launchSequenceStep++;
                break;
            case 2:
                if (launcher.getVelocity() > velocity - 20) {
                    wallOpen();
                    intakeIn();
                    launchTimer.reset();
                    launchSequenceStep++;
                }
                break;
            case 3:
                if (launchTimer.seconds() > 1.75) {
                    intakeStop();
                    wallClose();
                    launcher.setVelocity(idleVelocity);
                    launcher2.setVelocity(idleVelocity);
                    launchSequenceStep = 1;
                    launching = false;
                }
                break;
        }

    }

    public boolean getLaunchStatus() {
        return launching;
    }

    public void startLaunch() {
        launching = true;
    }

    public void stopLaunch() {
        launching = false;
        intakeStop();
        wallClose();
        launcher.setVelocity(idleVelocity);
        launcher2.setVelocity(idleVelocity);
    }


    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx intake, kicker, launcher, launcher2;
    public Servo vision, vision1, wall;
    public Limelight3A limelight;
    public ColorSensor intakeSensor;
    public ColorSensor transferSensor;

    public boolean intakeIsOpen;
    public boolean intaking;
    private boolean launching = false;
    private int launchSequenceStep = 1;

    // ================= LIGHT VALUES =================
    public final double OFF = 0.0;
    public final double GREEN = 0.475;
    public final double ORANGE = 0.333;

    public boolean intakeFull = false;
    public final double COLORIntake_THRESHOLD = 45;
    public final double COLORTransfer_THRESHOLD = 85;

    // ==================Intake Initialization ==========================`
    public long intakeStopTime = 0;
    public boolean wasIntaking = false;
    public final long REVERSE_TIME_MS = 100;


    /**
     * @param robotPose
     * @return heading in radians
     */
    public double blueGetGoalHeading(Pose robotPose) {
        Pose goal = new Pose(5, 130);
        double opposite = goal.getY() - robotPose.getY();
        double adjacent = robotPose.getX() - goal.getX();
        double heading = Math.PI - Math.atan2(opposite, adjacent);
        return heading;
    }

    public double blueGetDistanceFromGoal(Pose robotPose) {
        double opposite = 144 - robotPose.getY();
        double adjacent = robotPose.getX();
        double hypot = Math.sqrt((opposite * opposite) + (adjacent * adjacent));
        return hypot;
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     *
     * @param robotPose
     * @return heading in radians
     */

    // todo work on this like thing above this blueGetGoalHeading
    public double redGetGoalHeading(Pose robotPose) {
        Pose goal = new Pose(144, 139);
        double opposite = goal.getY() - robotPose.getY();
        double adjacent = 144 - robotPose.getX();
        double heading = Math.atan2(opposite, adjacent);
        return heading;
    }

    public double redGetDistanceFromGoal(Pose robotPose) {
        double opposite = 144 - robotPose.getY();
        double adjacent = 144 - robotPose.getX();
        double hypot = Math.sqrt((opposite * opposite) + (adjacent * adjacent));
        return hypot;
    }


    public double calculateLaunchVelocity(double distance) {
        double x = distance;
        double velocity = (-0.000234073 * x * x * x + (0.101791 * x * x - 8.57328 * x + 1400)); // was 1374
        return velocity;
    }


    /**
     * Reads a Pose from file.
     * Returns null if the file does not exist or is invalid.
     */




    // todo BLUE SIDE UPDATED RED SIDE NOT UPDATED
    public final Pose blueStartFar = new Pose(55, 8.39, Math.toRadians(90));
    public final Pose blueFarScorePose = new Pose(52, 11, Math.toRadians(116));
    public final Pose blueFarPark = new Pose(35.6, 12.85, Math.toRadians(90));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallSetup = new Pose(16.6, 23, Math.toRadians(-89));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWall = new Pose(16.6, 2.2, Math.toRadians(-89));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallGetBack = new Pose(16.6, 2.5, Math.toRadians(-89));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup = new Pose(35.2, 4, Math.toRadians(-160));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer = new Pose(14, 3, Math.toRadians(-160));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup = new Pose(37.6, 15, Math.toRadians(173));
    public final Pose blueFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore = new Pose(10.3, 15, Math.toRadians(173));
    public final Pose blueStartClose = new Pose(16, 115.7435, Math.toRadians(139));
    public final Pose blueScorePose = new Pose(44, 86, Math.toRadians(132));
    public final Pose blueScorePose2 = new Pose(51.9182, 103.703, Math.toRadians(150.25));
    public final Pose bluePickup1Pose = new Pose(15, 75.5, Math.toRadians(180));
    public final Pose blueSetup1Pose = new Pose(40, 75.5, Math.toRadians(180));
    public final Pose bluePickup2Pose = new Pose(6.5, 51.5, Math.toRadians(180));
    public final Pose blueSetup2Pose = new Pose(43.576, 51.5, Math.toRadians(180));
    public final Pose blueGate = new Pose(19, 56, Math.toRadians(-90));
    public final Pose blueGateFacingGoalSetup = new Pose(19, 75.5, Math.toRadians(180));
    public final Pose blueGateFacingGoal = new Pose(13.5, 74.5, Math.toRadians(70));
    public final Pose blueAutoEnd = new Pose(30, 58, Math.toRadians(-90));
    public final Pose blueGateSetupPose = new Pose(17, 49.7, Math.toRadians(179.697));
    public final Pose bluePickup3Pose = new Pose(6.5, 27, Math.toRadians(179));
    public final Pose blueSetup3Pose = new Pose(43, 27, Math.toRadians(179));
    public final Pose blueGateHold = new Pose(9.25, 57.75, Math.toRadians(155.28));
    public final Pose blueGateHarvest = new Pose(8, 55, Math.toRadians(127));
    public final Pose blueGateHarvestSetup = new Pose(36.394, 54.5, Math.toRadians(155.7));


    public final Pose redStartFar = new Pose(89, 8.39, Math.toRadians(90));
    public final Pose redFarScorePose = new Pose(87, 13.7, Math.toRadians(65.5));
    public final Pose redFarPark = new Pose(110, 10, Math.toRadians(90));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallSetup = new Pose(144, 21, Math.toRadians(-90));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWall = new Pose(144, 0, Math.toRadians(-90));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallGetBack = new Pose(144, 2.5, Math.toRadians(-90));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerSetup = new Pose(114.7, 10, Math.toRadians(-20));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayer = new Pose(142, 9, Math.toRadians(-20));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMoreSetup = new Pose(110, 16, Math.toRadians(10));
    public final Pose redFarGrabFromHumanPlayerZoneOffTheWallFacingHumanPlayerTowardsGoalMore = new Pose(142, 18, Math.toRadians(10));
    public final Pose redStartClose = new Pose(129.2095, 122.4811, Math.toRadians(39.3531));
    public final Pose redScorePose = new Pose(92, 92, Math.toRadians(39));
    public final Pose redScorePose2 = new Pose(92, 103, Math.toRadians(34));
    public final Pose redPickup1Pose = new Pose(134, 82.2643, Math.toRadians(0.6));
    public final Pose redSetup1Pose = new Pose(107.6, 81.6, Math.toRadians(1.33));
    public final Pose redPickup2Pose = new Pose(141, 57.5, Math.toRadians(.08));
    public final Pose redSetup2Pose = new Pose(107.6, 58, Math.toRadians(.08));
    public final Pose redSetupGate = new Pose(133, 58, Math.toRadians(0));
    public final Pose redGateFacingParkingZone = new Pose(138, 62.5, Math.toRadians(-90));
    public final Pose redGateFacingParkingZone2 = new Pose(138, 58, Math.toRadians(-90));
    public final Pose redGateFacingGoal = new Pose(134, 76, Math.toRadians(90));
    public final Pose redGateFacingGoalSetup = new Pose(129, 82.26, Math.toRadians(0));
    public final Pose redPickup3Pose = new Pose(142, 34.5, Math.toRadians(.8));
    public final Pose redSetup3Pose = new Pose(107.6, 34.5, Math.toRadians(.8));
    public final Pose redAutoEnd = new Pose(129, 57.8, Math.toRadians(-90));
    public final Pose redGateHold = new Pose(137.4, 61.134, Math.toRadians(37));
    public final Pose redGateHarvest = new Pose(135.6, 60, Math.toRadians(46.5));
    public final Pose redGateHarvestSetup = new Pose(117.33, 60.8657, Math.toRadians(39.38));

    HardwareMap hwMap;

    public void init() {
        if (linearOpMode != null) {
            hwMap = linearOpMode.hardwareMap;
        } else {
            hwMap = opmode.hardwareMap;
        }

        // This is where we have to get stuff from the Driver Station
        frontLeftMotor = hwMap.get(DcMotorEx.class, "fl");
        backLeftMotor = hwMap.get(DcMotorEx.class, "bl");
        frontRightMotor = hwMap.get(DcMotorEx.class, "fr");
        backRightMotor = hwMap.get(DcMotorEx.class, "br");

        intake = hwMap.get(DcMotorEx.class, "intake");
        kicker = hwMap.get(DcMotorEx.class, "kicker");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher2 = hwMap.get(DcMotorEx.class, "launcher2");

        vision = hwMap.get(Servo.class, "vision");
        vision1 = hwMap.get(Servo.class, "vision1");
        wall = hwMap.get(Servo.class, "wall");
        intakeSensor = hwMap.get(ColorSensor.class, "color");
        transferSensor = hwMap.get(ColorSensor.class, "sensor");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Limelight Initialization
        limelight.start();

        // Motor Directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        kicker.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wheel Brakes
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher Initialization
        PIDFCoefficients pidf = new PIDFCoefficients(55, 0, 0, 15.5);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        launcher2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);


    }
}