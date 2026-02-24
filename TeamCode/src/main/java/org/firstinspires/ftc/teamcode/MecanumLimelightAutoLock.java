package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Mecanum Limelight Auto Lock")
public class MecanumLimelightAutoLock extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private Limelight3A limelight;

    private static final double TURN_KP = 0.02;
    private static final double MAX_AUTO_TURN = 0.5;
    private static final double DEADZONE = 1.0;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.5) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    double tx = result.getTx();

                    if (Math.abs(tx) > DEADZONE) {
                        turn = tx * TURN_KP;

                        turn = Math.max(-MAX_AUTO_TURN,
                                Math.min(MAX_AUTO_TURN, turn));
                    } else {
                        turn = 0;
                    }

                    telemetry.addData("TX", tx);
                }
            }

            double flPower = y + x + turn;
            double frPower = y - x - turn;
            double blPower = y - x + turn;
            double brPower = y + x - turn;

            double max = Math.max(1.0, Math.abs(flPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(blPower));
            max = Math.max(max, Math.abs(brPower));

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);

            telemetry.update();
        }
    }
}