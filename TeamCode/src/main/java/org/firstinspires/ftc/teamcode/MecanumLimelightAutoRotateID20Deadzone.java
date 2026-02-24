package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(name = "Mecanum Limelight AutoRotate ID20 Deadzone")
public class MecanumLimelightAutoRotateID20Deadzone extends LinearOpMode {

    // Mecanum motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

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

        // Adjust directions if needed
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- LIMELIGHT SETUP ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        telemetry.addLine("Ready! Press left trigger to auto-rotate to Tag 20");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Limelight Connected", limelight.isConnected());
            // --- GAMEPAD INPUT ---
            double y = -gamepad1.left_stick_y; // Forward/back
            double x = gamepad1.left_stick_x;  // Strafe left/right
            double manualRotation = gamepad1.right_stick_x * 0.7; // adjust sensitivity here
            double rotation = manualRotation;

            boolean leftTrigger = gamepad1.left_trigger > 0.1; // Auto-rotate trigger

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

                        boolean tag20Found = false;

                        for (LLResultTypes.FiducialResult fr : fiducials) {

                            telemetry.addData("Seen Tag ID", fr.getFiducialId());

                            if (fr.getFiducialId() == 20) {

                                tag20Found = true;

                                double tx = fr.getTargetXDegrees();
                                telemetry.addData("Tag20 TX", tx);

                                // --- DEADZONE & PROPORTIONAL ROTATION ---
                                if (Math.abs(tx) < deadzone) {
                                    rotation = 0;
                                    telemetry.addLine("Aligned (Inside Deadzone)");
                                }
                                else {
                                    rotation = -tx * kP;

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

                        if (!tag20Found) {
                            telemetry.addLine("Tag 20 NOT Found");
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