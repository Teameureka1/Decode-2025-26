package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.Config;

import java.util.List;

@TeleOp(name = "Silent")
public class Silent extends LinearOpMode {

    private Config robot;

    private Limelight3A limelight;
    private Servo vision, vision1, wall;

    private ColorSensor colorSensor;
    private ColorSensor sensor;
    private DcMotorEx launcher, launcher2;

    // ================= LIGHT VALUES =================
    private final double OFF = 0.0;
    private final double PURPLE = 0.7;
    private final double ORANGE = 0.333;

    private boolean colorLatched = false;
    private final double COLORIntake_THRESHOLD = 45;
    private final double COLORTransfer_THRESHOLD = 100;

    // ==================Intake ==========================
    private long intakeStopTime = 0;
    private boolean wasIntaking = false;
    private final long REVERSE_TIME_MS = 100;

    @Override
    public void runOpMode() {

        robot = new Config(this);
        robot.init();

        vision = hardwareMap.get(Servo.class, "vision");
        vision1 = hardwareMap.get(Servo.class, "vision1");
        wall = hardwareMap.get(Servo.class, "wall");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

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
                robot.intake.setVelocity(-1100);
                robot.kicker.setPower(-1);
                wasIntaking = true;

            } else if (intakeInput > 0.1) {
                // Intake out
                robot.intake.setVelocity(1100);
                robot.kicker.setPower(1);
                wasIntaking = true;

            } else {
                // Stick released
                if (wasIntaking) {
                    intakeStopTime = now;
                    wasIntaking = false;
                }

                // Run reverse for 150 ms after release
                if (now - intakeStopTime < REVERSE_TIME_MS) {
                    robot.intake.setVelocity(-1500);   // reverse burst
                } else {
                    robot.intake.setPower(0);
                    robot.kicker.setPower(0);
                }
            }
            // ================= WALL =================
            if (gamepad2.a) wall.setPosition(0.15);
            if (gamepad2.y) wall.setPosition(0.32);

            // ================= LAUNCHER =================
            if (gamepad2.right_trigger > 0.5) {
                robot.launcher.setVelocity(1300);
                robot.launcher2.setVelocity(1300);
            } else if (gamepad2.left_trigger > 0.5) {
                robot.launcher.setVelocity(1560);
                robot.launcher2.setVelocity(1560);
            } else {
                robot.launcher.setVelocity(0);
                robot.launcher2.setVelocity(0);
            }

            // ================= LIMELIGHT =================
            boolean locked = false;

            if (gamepad1.left_trigger > 0.1) {

                LLResult result = limelight.getLatestResult();

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

            // ================= COLOR SENSOR =================
            if (colorSensor.alpha() > COLORIntake_THRESHOLD && sensor.alpha() > COLORTransfer_THRESHOLD) {
                colorLatched = true;
            } else {
                colorLatched = false;
            }

            // ================= LIGHT PRIORITY SYSTEM =================
            if (locked) {

                vision.setPosition(PURPLE);
                vision1.setPosition(PURPLE);

            } else if (colorLatched) {

                vision.setPosition(ORANGE);
                vision1.setPosition(ORANGE);

            } else {

                vision.setPosition(OFF);
                vision1.setPosition(OFF);
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
            telemetry.addData("Intake Full", colorLatched);
            telemetry.addData("Intake Sensor", colorSensor.alpha());
            telemetry.addData("Transfer Sensor", sensor.alpha());
            telemetry.addData("Launch Velocity:", launcher.getVelocity());
            telemetry.addData("Launch2 Velocity:", launcher2.getVelocity());

            telemetry.update();
        }
    }
}