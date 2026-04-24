package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name = "Launcher Test")
public class Launcher extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotorEx launcher2;

    // ===== Constants =====

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        // ===== Proper setup =====
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Make them spin same direction physically
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Basic PIDF (stable baseline)
        PIDFCoefficients pidf = new PIDFCoefficients(10, 0, 0, 11);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        waitForStart();

        while (opModeIsActive()) {

            // ===== Controls =====

            if (gamepad1.a) {
                launcher.setVelocity(1000);
            } else {
                launcher.setVelocity(1000);
                launcher2.setVelocity(1000);
            }


            // ===== Telemetry =====
            telemetry.addData("L1 Velocity", launcher.getVelocity());
            telemetry.addData("L2 Velocity", launcher2.getVelocity());
            telemetry.update();
        }
    }
}