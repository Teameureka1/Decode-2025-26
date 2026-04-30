package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Configuration.Config;

import java.util.List;

@TeleOp(name = "Blue Far")
public class BlueTeleopFar extends LinearOpMode {

    private Config robot;

    // ================= LIGHT VALUES =================
    private final double OFF = 0.0;
    private final double GREEN = 0.475;
    private final double ORANGE = 0.333;

    private boolean intakeFull = false;
    private final double COLORIntake_THRESHOLD = 45;
    private final double COLORTransfer_THRESHOLD = 85;

    // ==================Intake Initialization ==========================`
    private long intakeStopTime = 0;
    private boolean wasIntaking = false;
    private final long REVERSE_TIME_MS = 50;

    @Override
    public void runOpMode() {

        robot = new Config(this);
        robot.init();
        robot.limelight.pipelineSwitch(9);

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.75;
            double speed = 0.3 + (0.7 * gamepad1.right_trigger);

            // ================= INTAKE =================
            double intakeInput = -gamepad2.left_stick_y;
            long now = System.currentTimeMillis();

            if (intakeInput < -0.1) {
                // Intake in
                robot.intake.setVelocity(-1000);
                robot.kicker.setPower(-1);
                wasIntaking = true;

            } else if (intakeInput > 0.1) {
                // Intake out
                robot.intake.setVelocity(1000);
                robot.kicker.setPower(1);
                wasIntaking = true;

            } else {
                // Stick released
                if (wasIntaking) {
                    intakeStopTime = now;
                    wasIntaking = false;
                }

                // Run reverse for 150 ms after release, because if we have 4 artifacts
                // it will spit the 4th one out.
                if (now - intakeStopTime < REVERSE_TIME_MS) {
                    robot.intake.setVelocity(-1500);   // reverse burst
                } else {
                    robot.intake.setPower(0);
                    robot.kicker.setPower(0);
                }
            }
            // ================= WALL =================
            if (gamepad2.a) robot.wall.setPosition(0.15);
            if (gamepad2.y) robot.wall.setPosition(0.32);

            // ================= LAUNCHER =================
            if (gamepad2.right_trigger > 0.5) {
                robot.launcher.setVelocity(1300);
                robot.launcher2.setVelocity(1300);
            } else if (gamepad2.left_trigger > 0.5) {
                robot.launcher.setVelocity(1620);
                robot.launcher2.setVelocity(1620);
            } else {
                robot.launcher.setVelocity(1000);
                robot.launcher2.setVelocity(1000);
            }

            // ================= LIMELIGHT =================
            boolean locked = false;

            if (gamepad1.left_trigger > 0.1) {

                LLResult result = robot.limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 20) {

                            double tx = fr.getTargetXDegrees();

                            if (Math.abs(tx) < 0.8) {
                                rotation = 0;
                                locked = true;
                            } else {
                                rotation = tx * 0.27;
                                rotation = Math.max(-0.55, Math.min(rotation, 0.55));
                            }
                        }
                    }
                }
            }

            // ================= COLOR SENSORS =================
            if (robot.intakeSensor.alpha() > COLORIntake_THRESHOLD && robot.transferSensor.alpha() > COLORTransfer_THRESHOLD) {
                intakeFull = true;
            } else {
                intakeFull = false;
            }

            // ================= LIGHT SYSTEM =================
            if (locked) {

                robot.vision.setPosition(GREEN);
                robot.vision1.setPosition(GREEN);

            } else if (intakeFull) {

                robot.vision.setPosition(ORANGE);
                robot.vision1.setPosition(ORANGE);

            } else {

                robot.vision.setPosition(OFF);
                robot.vision1.setPosition(OFF);
            }

            // ================= DRIVE =================
            double fl = (y + x + rotation) * speed;
            double bl = (y - x + rotation) * speed;
            double fr = (y - x - rotation) * speed;
            double br = (y + x - rotation) * speed;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(bl),
                            Math.max(Math.abs(fr), Math.abs(br))));

            if (max > 1) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            robot.frontLeftMotor.setPower(fl);
            robot.backLeftMotor.setPower(bl);
            robot.frontRightMotor.setPower(fr);
            robot.backRightMotor.setPower(br);

            // ================= TELEMETRY =================
            telemetry.addData("Locked", locked);
            telemetry.addData("Intake Full", intakeFull);
            telemetry.addData("Intake Sensor", robot.intakeSensor.alpha());
            telemetry.addData("Transfer Sensor", robot.transferSensor.alpha());
            telemetry.addData("Launch Velocity:", robot.launcher.getVelocity());
            telemetry.addData("Launch2 Velocity:", robot.launcher2.getVelocity());
            telemetry.update();
        }
    }
}