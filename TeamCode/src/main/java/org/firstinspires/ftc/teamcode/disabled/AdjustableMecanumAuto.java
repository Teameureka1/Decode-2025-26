package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AdjustableMecanumAuto extends LinearOpMode {

    DcMotor fl, bl, fr, br;

    // --- GoBilda 5202 constants ---
    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double GEAR_RATIO = 1.0;

    @Override
    public void runOpMode() {

        // Motor setup
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        // Reverse left side for forward movement
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        // BRAKE mode
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // --- SEQUENCE OF MOVEMENTS ---
        moveForward(.1, 12);   // forward at 50% power for 12 inches
        moveForward(.3, 12);   // forward at 20% power for 12 inches
        moveForward(0.1, 24);   // forward at 70% power for 24 inches
        moveBackward(0.3, 6);   // backward at 30% power for 6 inches

        sleep(500);
    }

    // --- Drive forward ---
    public void moveForward(double power, double inches) {
        int ticks = inchesToTicks(inches);

        fl.setTargetPosition(fl.getCurrentPosition() + ticks);
        bl.setTargetPosition(bl.getCurrentPosition() + ticks);
        fr.setTargetPosition(fr.getCurrentPosition() + ticks);
        br.setTargetPosition(br.getCurrentPosition() + ticks);

        runToPosition(power);
    }

    // --- Drive backward ---
    public void moveBackward(double power, double inches) {
        int ticks = inchesToTicks(inches);

        fl.setTargetPosition(fl.getCurrentPosition() - ticks);
        bl.setTargetPosition(bl.getCurrentPosition() - ticks);
        fr.setTargetPosition(fr.getCurrentPosition() - ticks);
        br.setTargetPosition(br.getCurrentPosition() - ticks);

        runToPosition(power);
    }

    // --- Helper to run to target ---
    private void runToPosition(double power) {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        while (opModeIsActive() &&
                (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {
            telemetry.addData("Status", "Moving...");
            telemetry.update();
        }

        // Stop motors
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Convert inches to encoder ticks ---
    private int inchesToTicks(double inches) {
        double rotations = inches / (Math.PI * WHEEL_DIAMETER_INCHES);
        return (int)(rotations * TICKS_PER_REV * GEAR_RATIO);
    }
}