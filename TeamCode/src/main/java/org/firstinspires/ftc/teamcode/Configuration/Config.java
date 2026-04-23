package org.firstinspires.ftc.teamcode.Configuration;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    private LinearOpMode opmode;
    public Config(LinearOpMode linearOpMode) {
        opmode = linearOpMode;
    }


    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx intake, kicker, launcher, launcher2;
    public Servo wall;
    public Limelight3A limelight;

    public void init() {
        HardwareMap hwMap = opmode.hardwareMap;

        frontLeftMotor  = hwMap.get(DcMotor.class, "fl");
        backLeftMotor   = hwMap.get(DcMotor.class, "bl");
        frontRightMotor = hwMap.get(DcMotor.class, "fr");
        backRightMotor  = hwMap.get(DcMotor.class, "br");

        intake   = hwMap.get(DcMotorEx.class, "intake");
        kicker   = hwMap.get(DcMotorEx.class, "kicker");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher2 = hwMap.get(DcMotorEx.class, "launcher2");

        wall = hwMap.get(Servo.class, "wall");


        limelight = hwMap.get(Limelight3A.class, "limelight");

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

        // Limelight Initialization
        limelight.pipelineSwitch(8);
        limelight.start();

    }
}
