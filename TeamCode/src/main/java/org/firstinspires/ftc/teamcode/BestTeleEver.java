package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.BreakIterator;

@TeleOp(name = "BestTeleEver")
public class BestTeleEver extends LinearOpMode {

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
            double throttle = Range.clip(gamepad1.right_trigger + .2, 0, 1);
            double intakeWheelInput = -gamepad2.left_stick_y;
            double intakeStringInput = -gamepad2.right_stick_y;
            double launcherInputFar = gamepad2.right_trigger;
            double launcherInputClose = gamepad2.left_trigger;


            boolean kickerInput = gamepad2.y;
            boolean wallHoldInput = gamepad2.x;
            boolean wallReleaseInput = gamepad2.b;
            boolean angleFarInput = gamepad2.left_bumper;
            boolean angleCloseInput = gamepad2.right_bumper;

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
            if (intakeWheelInput > .1) {
                intakeWheels.setPower(1);
            } else if (intakeWheelInput < -.1) {
                intakeWheels.setPower(-1);
            } else {
                intakeWheels.setPower(0);
            }
            // -------------------------Intake String-------------------------------------------
            if (intakeStringInput > .1) {
                intakeString.setPower(1);
            }
            else {
                intakeString.setPower(0);
            }
            //-------------------------Launcher------------------------------------------------
            // ---------------Far-----------------------
            if (launcherInputFar > .5) {
                launcher.setVelocity(1730);
            }
            // ----------Close-----------
            else if (launcherInputClose > .5) {
                launcher.setVelocity(1410);
            }
            //-----------Off-----------
            else {
                launcher.setVelocity(0);
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
                angle.setPosition(.68);
            }
            else if (angleCloseInput) {
                angle.setPosition(.6);
            }
        }
    }
}
