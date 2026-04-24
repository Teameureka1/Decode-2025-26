package org.firstinspires.ftc.teamcode.Configuration;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private LinearOpMode opmode;
    public Config(LinearOpMode linearOpMode) {
        opmode = linearOpMode;
    }


    public DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx intake, kicker, launcher, launcher2;
    public Servo vision, vision1, wall;
    public Limelight3A limelight;
    public ColorSensor intakeSensor;
    public ColorSensor transferSensor;

    public void init() {
        HardwareMap hwMap = opmode.hardwareMap;
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
