package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagAutoRotate extends OpMode {
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private final MecanumDrive drive = new MecanumDrive();


    double kp = .002;
    double error = 0;
    double lastError = 0;
    double goalX = 0;
    double angleTolerance = .2;
    double kd = .0001;
    double curTime = 0;
    double lastTime = 0;

    double forward, strafe, rotate;

    double[] stepSizes = {1.0, 0.1, .01, .001, .0001};

    int stepIndex = 2;
    @Override
    public void init() {
    aprilTagWebcam.init(hardwareMap, telemetry);
    drive.init(hardwareMap,false);

    telemetry.addLine("Initialized");
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        if (gamepad1.left_trigger > .5) {
            if (id24 != null) {
                error = goalX - id24.ftcPose.bearing;

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;

                } else {
                    double pTerm = error * kp;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kd;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }


        } else {
            lastTime = getRuntime();
            lastError = 0;

        }

        drive.drive(forward, strafe, rotate);

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kp -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kd += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kd -= stepSizes[stepIndex];
        }

        if (id24 != null) {
            if (gamepad1.left_trigger > .5) {
                telemetry.addLine("Auto Align");
            }
            aprilTagWebcam.displayDetectionTelemetry(id24);
            telemetry.addData("error", error);
        }



        else {
            telemetry.addLine("Manual Rotate Mode");
        }
        telemetry.addLine("------------------------------------------------");
        telemetry.addData("Tuning P", "%.4f D-pad U/D", kp);
        telemetry.addData("Tuning D", "%.4f D-pad L/R", kd);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}
