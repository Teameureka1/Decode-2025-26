package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.Config;

import java.util.List;

@TeleOp(name = "blueFerdinand")
public class blueTeleop extends OpMode {

    private Config robot;
    boolean intakeIsOn = false;

    @Override
    public void init() {

        robot = new Config(this);
        robot.init();
        robot.limelight.pipelineSwitch(8);

    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x * 0.75;
        double speed = 0.3 + (0.7 * gamepad1.right_trigger);

        // ================= LAUNCHER =================
        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.launchThreeUpdater();

        if (gamepad2.bWasPressed()) {
            intakeIsOn = !intakeIsOn;
            robot.stopLaunch();
            if (intakeIsOn) {
                robot.intakeIn();
            } else {
                robot.intakeStop();
            }

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
        if (robot.intakeSensor.alpha() > robot.COLORIntake_THRESHOLD && robot.transferSensor.alpha() > robot.COLORTransfer_THRESHOLD) {
            robot.intakeFull = true;
        } else {
            robot.intakeFull = false;
        }

        // ================= LIGHT SYSTEM =================
        if (locked) {

            robot.vision1.setPosition(robot.GREEN);

        } else if (robot.intakeFull) {

            robot.vision.setPosition(robot.ORANGE);

        } else {

            robot.vision.setPosition(robot.OFF);
            robot.vision1.setPosition(robot.OFF);
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
        telemetry.addData("Intake Full", robot.intakeFull);
        telemetry.addData("Intake Sensor", robot.intakeSensor.alpha());
        telemetry.addData("Transfer Sensor", robot.transferSensor.alpha());
        telemetry.addData("Launch Velocity:", robot.launcher.getVelocity());
        telemetry.addData("Launch2 Velocity:", robot.launcher2.getVelocity());
        telemetry.update();
    }
}