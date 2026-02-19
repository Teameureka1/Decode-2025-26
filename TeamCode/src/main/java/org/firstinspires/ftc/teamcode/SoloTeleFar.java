package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "SoloTeleFar")
public class SoloTeleFar extends LinearOpMode {

    private DcMotorEx fl;
    private DcMotorEx br;
    private DcMotorEx bl;
    private DcMotorEx fr;
    private DcMotorEx intakeWheels;
    private DcMotorEx intakeString;
    private DcMotorEx launcher;

    private Servo kicker;
    private Servo wall;
    private Servo angle;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        intakeWheels = hardwareMap.get(DcMotorEx.class, "intake wheels");
        intakeString = hardwareMap.get(DcMotorEx.class, "intake string");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(55, 0, 0, 14.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        kicker = hardwareMap.get(Servo.class, "kicker");
        wall = hardwareMap.get(Servo.class, "wall");
        angle = hardwareMap.get(Servo.class, "angle");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeWheels.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeString.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Waiting for start
        while (opModeIsActive()) {

            double max;

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double throttle = Range.clip(gamepad1.right_trigger + .55, 0, 1);
            boolean intakeWheelInput = gamepad1.left_bumper;
            boolean intakeStringInput = gamepad1.right_bumper;
            double launcherInputClose = gamepad2.right_trigger;
            double launcherInputFar = gamepad1.left_trigger;



            boolean kickerInput = gamepad1.y;
            boolean wallHoldInput = gamepad1.x;
            boolean wallReleaseInput = gamepad1.b;
            boolean angleFarInput = gamepad1.dpad_up;
            boolean angleCloseInput = gamepad1.dpad_down;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));


            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;

            }
            fl.setPower(frontLeftPower * throttle);
            fr.setPower(frontRightPower * throttle);
            bl.setPower(backLeftPower * throttle);
            br
                    .setPower(backRightPower * throttle);
            //-----------------------------Intake Wheels-------------------------------------
            if (intakeWheelInput) {
                intakeWheels.setPower(1);
            } else if (intakeWheelInput) {
                intakeWheels.setPower(-1);
            } else {
                intakeWheels.setPower(0);
            }
            // -------------------------Intake String-------------------------------------------
            if (intakeStringInput) {
                intakeString.setPower(1);
            }
            else {
                intakeString.setPower(0);
            }
            //-------------------------Launcher------------------------------------------------
            // ---------------Far-----------------------
            if (launcherInputFar > .5) {
                launcher.setVelocity(1760);
            }
            //--------------Close----------------
            else if (launcherInputClose > .5) {
                launcher.setVelocity(1460);
            }
            else {
                launcher.setVelocity(1000);
            }
            // -----------------------------------Kicker------------------------------------------
            if (kickerInput) {
                kicker.setPosition(.55);
            } else {
                kicker.setPosition(.25);
            }
            //------------------------------------Wall---------------------------------------------
            if (wallReleaseInput) {
                wall.setPosition(.94);
            } else if (wallHoldInput) {
                wall.setPosition(.86);
            }
            //-------------------------------------Angle------------------------------------------
            if (angleFarInput) {
                angle.setPosition(.63);
            }
            else if (angleCloseInput) {
                angle.setPosition(.6);
            }
        }
    }
}
