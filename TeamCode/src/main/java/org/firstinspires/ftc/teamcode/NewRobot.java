package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "New Robot")
public class NewRobot extends LinearOpMode {
    //------------------------CODE NAMES-----------------------------
    private DcMotorEx intake;
    private DcMotorEx kicker;
    private DcMotorEx launcher;

    private DcMotorEx fl;
    private DcMotorEx br;
    private DcMotorEx bl;
    private DcMotorEx fr;

    Servo wall;


    public void runOpMode() throws InterruptedException {

        //--------------------CONFIGURATION---------------------------
        wall = hardwareMap.get(Servo.class, "wall");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(55, 0, 0, 14.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");

        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        //--------------------DIRECTION--------------------------------
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        kicker.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);

        //---------------------BRAKES-----------------------------------
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //-------------------START OF OPMODE--------------------------

        waitForStart();
        while (opModeIsActive()) {

        //-------------------GAMEPAD INPUTS---------------------------
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double intakeInput = gamepad2.left_stick_y;
            double intakeBackwardsInput = -gamepad2.left_stick_y;
            double launcherInput = gamepad2.right_trigger;
            double launcherInputFar = gamepad2.left_trigger;
            double throttle = Range.clip(gamepad1.right_trigger + .22, 0, 1);

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
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
            br.setPower(backRightPower * throttle);
            //---------------------INTAKE----------------------------
            if (intakeBackwardsInput > .05) {
                intake.setPower(1);
                kicker.setPower(1);
            } else if (intakeInput > .1) {
                intake.setPower(-.6);
                kicker.setPower(-1);
            } else {
                intake.setPower(0);
                kicker.setPower(0);
            }
            //-------------------------Launcher------------------------------------------------
            // ---------------Close-----------------------
            if (launcherInput > .35) {
                launcher.setVelocity(1375);
            }
            //-----------------Far-------------------------
            else if (launcherInputFar > .35) {
                launcher.setVelocity(1615);
            }
            else {
                launcher.setVelocity(0);
            }
            if (gamepad2.a) {
                wall.setPosition(.5);
            }
            if (gamepad2.y) {
                wall.setPosition(.68);
            }

        }
    }
}
