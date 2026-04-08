package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class LimelightID4 extends LinearOpMode {

    // Mecanum motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx intake;
    private DcMotorEx kicker;
    private DcMotorEx launcher;
    private DcMotorEx launcher2;


    // Limelight 3A
    private Limelight3A limelight;


    // Auto-rotate tuning
    private final double kP       = 0.02;  // Proportional gain
    private final double minPower = 0.06;  // Minimum rotational power
    private final double maxPower = 0.4;   // Maximum rotational power
    private final double deadzone = 0.8;   // Degrees deadzone for rotation

    @Override
    public void runOpMode() throws InterruptedException {

        // --- INIT MOTORS ---
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backRightMotor  = hardwareMap.get(DcMotor.class, "br");
        Servo wall = hardwareMap.get(Servo.class, "wall");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(DcMotorEx.class, "kicker");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");


        // Adjust directions if needed
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        kicker.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(55, 0, 0, 14.5);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcher2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- LIMELIGHT SETUP ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8);
        limelight.start();

        telemetry.addLine("Ready! Press left trigger to auto-rotate to Tag 20");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Limelight Connected", limelight.isConnected());
            // --- GAMEPAD INPUT ---
            double y = -gamepad1.left_stick_y; // Forward/back
            double x = gamepad1.left_stick_x;  // Strafe left/right
            double manualRotation = gamepad1.right_stick_x * .7; // adjust sensitivity here
            double rotation = manualRotation;

            boolean leftTrigger = gamepad1.left_trigger > 0.1; // Auto-rotate trigger
            double intakeInput = gamepad2.left_stick_y;
            double intakeBackwardsInput = -gamepad2.left_stick_y;
            double launcherInput = gamepad2.right_trigger;
//---------------------INTAKE----------------------------
            if (intakeBackwardsInput > .05) {
                intake.setPower(1);
                kicker.setPower(1);
            } else if (intakeInput > .1) {
                intake.setPower(-1);
                kicker.setPower(-1);
            } else {
                intake.setPower(0);
                kicker.setPower(0);
            }
            if (gamepad2.a) {
                wall.setPosition(.15);
            }
            if (gamepad2.y) {
                wall.setPosition(.32);
            }
            //-------------------------Launcher------------------------------------------------
            // ---------------Close-----------------------
            if (launcherInput > .35) {
                launcher.setVelocity(1375);
                launcher2.setVelocity(1375);
            } else  {
                launcher2.setVelocity(0);
                launcher.setVelocity(0);

            }
            if (leftTrigger) {

                telemetry.addLine("=== AUTO ROTATE ACTIVE ===");

                LLResult result = limelight.getLatestResult();

                if (result == null) {
                    telemetry.addLine("Limelight: NO DATA (null result)");
                    rotation = 0; // prevent rotation if camera not responding
                }
                else if (!result.isValid()) {
                    telemetry.addLine("Limelight: Running, No Targets");
                    rotation = 0; // no target seen
                }
                else {

                    telemetry.addLine("Limelight: VALID Target Data");

                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    if (fiducials == null || fiducials.size() == 0) {
                        telemetry.addLine("No AprilTags Detected");
                        rotation = 0;
                    }
                    else {

                        boolean tag4Found = false;

                        for (LLResultTypes.FiducialResult fr : fiducials) {

                            telemetry.addData("Seen Tag ID", fr.getFiducialId());

                            if (fr.getFiducialId() == 4) {

                                tag4Found = true;

                                double tx = fr.getTargetXDegrees();
                                telemetry.addData("Tag4 TX", tx);

                                // --- DEADZONE & PROPORTIONAL ROTATION ---
                                if (Math.abs(tx) < deadzone) {
                                    rotation = 0;
                                    telemetry.addLine("Aligned (Inside Deadzone)");
                                }
                                else {
                                    rotation = tx * kP;

                                    // Minimum rotation power
                                    if (Math.abs(rotation) < minPower) {
                                        rotation = Math.signum(rotation) * minPower;
                                    }

                                    // Clamp maximum rotation power
                                    if (Math.abs(rotation) > maxPower) {
                                        rotation = Math.signum(rotation) * maxPower;
                                    }

                                    telemetry.addData("Rotation Power", rotation);
                                }

                                break; // stop once Tag 20 is handled
                            }
                        }

                        if (!tag4Found) {
                            telemetry.addLine("Tag4 NOT Found");
                            rotation = 0; // fallback
                        }
                    }
                }

                telemetry.update();
            }

            // --- MECANUM DRIVE CALCULATION ---
            double fl = y + x + rotation;
            double bl = y - x + rotation;
            double fr = y - x - rotation;
            double br = y + x - rotation;

            // Normalize powers
            double maxPow = Math.max(Math.abs(fl), Math.abs(bl));
            maxPow = Math.max(maxPow, Math.abs(fr));
            maxPow = Math.max(maxPow, Math.abs(br));
            if (maxPow > 1.0) {
                fl /= maxPow;
                bl /= maxPow;
                fr /= maxPow;
                br /= maxPow;
            }

            // Set motor powers
            frontLeftMotor.setPower(fl);
            backLeftMotor.setPower(bl);
            frontRightMotor.setPower(fr);
            backRightMotor.setPower(br);
        }
        telemetry.update();
    }
}