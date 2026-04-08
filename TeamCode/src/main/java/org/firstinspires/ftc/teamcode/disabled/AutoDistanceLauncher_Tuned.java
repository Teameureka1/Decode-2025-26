package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class AutoDistanceLauncher_Tuned extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx intake, kicker, launcher, launcher2;
    private Limelight3A limelight;

    // Intake burst
    boolean reverseBurstActive = false;
    double reverseStartTime = 0;
    boolean intakeWasUp = false;

    // Auto-rotate tuning
    private final double kP = 0.035;
    private final double minPower = 0.23;
    private final double maxPower = 0.4;
    private final double deadzone = 0.6;

    // Distance (CM)
    private final double CAMERA_HEIGHT = 25.85;
    private final double CAMERA_ANGLE = 19;
    private final double GOAL_HEIGHT = 74.95;

    // Launcher tuning (new)
    private final double BASE_VELOCITY = 900;
    private final double DISTANCE_MULTIPLIER = 2.3; // tuned: 150cm -> 1300
    private final double MIN_VELOCITY = 1000;
    private final double MAX_VELOCITY = 1680;

    // Smoothing (prevents jumpy RPM)
    private double currentVelocity = 1000;
    private final double SMOOTHING = 0.15;

    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backRightMotor  = hardwareMap.get(DcMotor.class, "br");

        intake   = hardwareMap.get(DcMotorEx.class, "intake");
        kicker   = hardwareMap.get(DcMotorEx.class, "kicker");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        Servo wall = hardwareMap.get(Servo.class, "wall");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        kicker.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(55, 0, 0, 14.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // === DRIVE INPUT ===
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x * 0.7;

            double throttle = gamepad1.right_trigger;
            double speedMultiplier = 0.3 + (0.7 * throttle);

            // === DISTANCE ===
            double distance = -1;
            LLResult llResult = limelight.getLatestResult();

            if (llResult != null && llResult.isValid()) {
                distance = getDistance(llResult.getTy());
                telemetry.addData("Distance (cm)", distance);
            } else {
                telemetry.addData("Target", "NOT FOUND");
            }

            // === TARGET VELOCITY (NEW FORMULA) ===
            double targetVelocity;

            if (distance > 0) {
                targetVelocity = BASE_VELOCITY + (distance * DISTANCE_MULTIPLIER);
            } else {
                targetVelocity = MIN_VELOCITY; // fallback
            }

            // Clamp velocity
            targetVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, targetVelocity));

            // Smooth velocity
            currentVelocity += (targetVelocity - currentVelocity) * SMOOTHING;

            launcher.setVelocity(currentVelocity);
            launcher2.setVelocity(currentVelocity);

            telemetry.addData("Target Vel", targetVelocity);
            telemetry.addData("Actual Vel", currentVelocity);

            // === INTAKE ===
            double intakeY = gamepad2.left_stick_y;
            boolean intakeUp = intakeY < -0.1;

            if (!intakeUp && intakeWasUp && !reverseBurstActive) {
                reverseBurstActive = true;
                reverseStartTime = runTime.milliseconds();
            }

            if (reverseBurstActive) {
                if (runTime.milliseconds() - reverseStartTime < 150) {
                    intake.setPower(1);
                } else {
                    reverseBurstActive = false;
                    intake.setPower(0);
                    kicker.setPower(0);
                }
            } else {
                if (intakeY < -0.1) {
                    intake.setPower(-1);
                    kicker.setPower(-1);
                } else if (intakeY > 0.1) {
                    intake.setPower(1);
                    kicker.setPower(1);
                } else {
                    intake.setPower(0);
                    kicker.setPower(0);
                }
            }

            intakeWasUp = intakeUp;

            // === WALL SERVO ===
            if (gamepad2.a) wall.setPosition(.15);
            if (gamepad2.y) wall.setPosition(.32);

            // === AUTO ROTATE ===
            if (gamepad1.left_trigger > 0.1) {

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getFiducialId() == 20) {

                            double tx = fr.getTargetXDegrees();

                            if (Math.abs(tx) < deadzone) {
                                rotation = 0;
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

            frontLeftMotor.setPower(fl);
            backLeftMotor.setPower(bl);
            frontRightMotor.setPower(fr);
            backRightMotor.setPower(br);

            telemetry.update();
        }
    }

    // === DISTANCE FUNCTION ===
    public double getDistance(double ty) {
        double angleToTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT;
        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }
}