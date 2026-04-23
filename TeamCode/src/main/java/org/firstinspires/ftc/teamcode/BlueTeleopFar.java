package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configuration.Config;

import java.util.List;

@TeleOp(name = "BlueTeleopFar")
public class BlueTeleopFar extends LinearOpMode {

    private Config robot;
    public Limelight3A limelight;
    public Servo vision;
    public Servo vision1;
    public final double indicatorLightVision = .3175, indicatorLightVisionIntake = 0.45, indicatorLightOff = 0;

    boolean reverseBurstActive = false;
    double reverseStartTime = 0;
    boolean intakeWasUp = false;

    private final double kP = .27;
    private final double minPower = .4;
    private final double maxPower = .55;
    private final double deadzone = 0.8;

    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Config(this);
        robot.init();
        vision = hardwareMap.get(Servo.class, "vision");
        vision1 = hardwareMap.get(Servo.class, "vision1");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Limelight Initialization
        limelight.pipelineSwitch(9);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.75;

            double throttle = gamepad1.right_trigger;
            double speedMultiplier = 0.3 + (0.7 * throttle);

            // === INTAKE ===
            double intakeY = -gamepad2.left_stick_y;
            boolean intakeUp = intakeY < -0.1;

            if (!intakeUp && intakeWasUp && !reverseBurstActive) {
                reverseBurstActive = true;
                reverseStartTime = runTime.milliseconds();
            }

            if (reverseBurstActive) {
                if (runTime.milliseconds() - reverseStartTime < 150) {
                    robot.intake.setPower(1);
                } else {
                    reverseBurstActive = false;
                    robot.intake.setPower(0);
                    robot.kicker.setPower(0);
                }
            } else {
                if (intakeY < -0.1) {
                    robot.intake.setPower(-.585);
                    robot.kicker.setPower(-1);
                } else if (intakeY > 0.1) {
                    robot.intake.setPower(.585);
                    robot.kicker.setPower(1);
                } else {
                    robot.intake.setPower(0);
                    robot.kicker.setPower(0);
                }
            }
            intakeWasUp = intakeUp;

            // === WALL SERVO ===
            if (gamepad2.a) robot.wall.setPosition(.15);
            if (gamepad2.y) robot.wall.setPosition(.32);

            // === LAUNCHER ===
            if (gamepad2.right_trigger > 0.35) {
                robot.launcher.setVelocity(1260);
                robot.launcher2.setVelocity(1260);
            } else if (gamepad2.left_trigger > 0.35) {
                robot.launcher.setVelocity(1600);
                robot.launcher2.setVelocity(1600);
            } else {
                robot.launcher.setVelocity(1140);
                robot.launcher2.setVelocity(1140);
            }

            if (gamepad2.a && gamepad2.b && gamepad2.y && gamepad2.x) {
                requestOpModeStop();
            }

            // === LIMELIGHT + AUTO ROTATE + LIGHT (TRIGGER CONTROLLED) ===
            boolean tagAligned = false;

            if (gamepad1.left_trigger > 0.1) {

                LLResult result = robot.limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 20) {

                            double tx = fr.getTargetXDegrees();

                            if (Math.abs(tx) < deadzone) {
                                rotation = 0;
                                tagAligned = true;
                            } else {
                                rotation = tx * kP;

                                if (Math.abs(rotation) < minPower)
                                    rotation = Math.signum(rotation) * minPower;

                                if (Math.abs(rotation) > maxPower)
                                    rotation = Math.signum(rotation) * maxPower;
                            }
                            break;
                        }
                    }
                }
            }

            // === LIGHT ONLY WHEN ALIGNED AND TRIGGER HELD ===
            if (tagAligned && gamepad1.left_trigger > 0.1) {
                vision.setPosition(indicatorLightVision);
                vision1.setPosition(indicatorLightVision);
            } else {
                vision.setPosition(indicatorLightOff);
                vision1.setPosition(indicatorLightOff);
            }

            // === MECANUM DRIVE ===
            double fl = (y + x + rotation) * speedMultiplier;
            double bl = (y - x + rotation) * speedMultiplier;
            double fr = (y - x - rotation) * speedMultiplier;
            double br = (y + x - rotation) * speedMultiplier;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                    Math.max(Math.abs(fr), Math.abs(br)));

            if (max > 1.0) {
                fl /= max;
                bl /= max;
                fr /= max;
                br /= max;
            }

            robot.frontLeftMotor.setPower(fl);
            robot.backLeftMotor.setPower(bl);
            robot.frontRightMotor.setPower(fr);
            robot.backRightMotor.setPower(br);
        }
    }
}