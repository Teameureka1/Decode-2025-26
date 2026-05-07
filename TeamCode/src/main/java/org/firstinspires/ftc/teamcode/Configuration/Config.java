package org.firstinspires.ftc.teamcode.Configuration;

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
// In this class basically if you go to another class and say "Config robot" it will take
// all of this class and put it there, but remember to add robot.init inside your init.
// You also you need "robot = new Config(this);" before robot.init.

public class Config {
    private LinearOpMode linearOpMode;
    private OpMode opmode;
    public Config(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public Config(OpMode opmode) {
        this.opmode = opmode;
    }

    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx intake, kicker, launcher, launcher2;
    public Servo vision, vision1, wall;
    public Limelight3A limelight;
    public ColorSensor intakeSensor;
    public ColorSensor transferSensor;

    public boolean intakeIsOpen;

// BLUE SIDE UPDATED RED SIDE NOT UPDATED
    public final Pose blueStartFar = new Pose(55, 8.39, Math.toRadians(90));
    public final Pose blueStartClose = new Pose(16.37, 115.9435, Math.toRadians(141));
    public final Pose blueScorePose = new Pose(50.84,81.467,Math.toRadians(136));
    public final Pose blueScorePose2 = new Pose(51.9182,103.703,Math.toRadians(150.25));
    public final Pose bluePickup1Pose = new Pose(15.2671,75, Math.toRadians(180));
    public final Pose blueSetup1Pose = new Pose(40,75, Math.toRadians(180));
    public final Pose bluePickup2Pose = new Pose(9.5,51, Math.toRadians(180));
    public final Pose blueSetup2Pose = new Pose(43.576,51, Math.toRadians(180));
    public final Pose blueGate = new Pose(19,56,Math.toRadians(-90));
    public final Pose blueAutoEnd = new Pose(30, 58, Math.toRadians(-90));
    public final Pose blueGateSetupPose = new Pose(17,49.7, Math.toRadians(179.697));
    public final Pose bluePickup3Pose = new Pose(8.5,26.5, Math.toRadians(179));
    public final Pose blueSetup3Pose = new Pose(43,26.5, Math.toRadians(179));

    public final Pose redStartFar = new Pose(89, 8.39, Math.toRadians(90));
    public final Pose redStartPose = new Pose(129.2095, 122.4811, Math.toRadians(39.3531));
    public final Pose redScorePose = new Pose(94.4201,92.2024,Math.toRadians(44.547));
    public final Pose redScorePose2 = new Pose(95.1933,107.5592,Math.toRadians(34));
    public final Pose redPickup1Pose = new Pose(134, 82.2643, Math.toRadians(0.6));
    public final Pose redSetup1Pose = new Pose(107.6,81.6, Math.toRadians(1.33));
    public final Pose redPickup2Pose = new Pose(141,58, Math.toRadians(.08));
    public final Pose redSetup2Pose = new Pose(107.6,58, Math.toRadians(.08));
    public final Pose redSetupGate = new Pose(135,58.4,Math.toRadians(0));
    public final Pose redGateFacingParkingZone = new Pose(138,61,Math.toRadians(-90));
    public final Pose redGateFacingGoal = new Pose(134,76,Math.toRadians(90));
    public final Pose redPickup3Pose = new Pose(141,33, Math.toRadians(.8));
    public final Pose redSetup3Pose = new Pose(107.6,33, Math.toRadians(.8));
    public final Pose redAutoEnd = new Pose(129,57.8, Math.toRadians(-90));
    public final Pose redGateHold = new Pose(137.4,61.134, Math.toRadians(37));
    public final Pose redGateHarvest = new Pose(135.6,60, Math.toRadians(46.5));
    public final Pose redGateHarvestSetup = new Pose(117.33,60.8657, Math.toRadians(39.38));





    HardwareMap hwMap;
    public void init() {
        if (linearOpMode != null) {
            hwMap = linearOpMode.hardwareMap;
        } else {
            hwMap = opmode.hardwareMap;
        }

        // This is where we have to get stuff from the Driver Station
        frontLeftMotor  = hwMap.get(DcMotorEx.class, "fl");
        backLeftMotor   = hwMap.get(DcMotorEx.class, "bl");
        frontRightMotor = hwMap.get(DcMotorEx.class, "fr");
        backRightMotor  = hwMap.get(DcMotorEx.class, "br");

        intake   = hwMap.get(DcMotorEx.class, "intake");
        kicker   = hwMap.get(DcMotorEx.class, "kicker");
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
