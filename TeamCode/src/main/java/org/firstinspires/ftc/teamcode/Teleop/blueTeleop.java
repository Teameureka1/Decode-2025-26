package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "blueFerdinand")
public class blueTeleop extends OpMode {

    Follower follower;
    private Config robot;
    boolean intakeIsOn = false;

    @Override
    public void init() {

        robot = new Config(this);
        robot.init();
        robot.limelight.pipelineSwitch(8);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(robot.blueStartFar);
    }
    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        double rotation = gamepad1.right_stick_x * 0.75;


        // ================= LAUNCHER =================
        if (gamepad2.yWasPressed()) {
            robot.startLaunch();
        }

        robot.launchThreeUpdater(follower);

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
        // =================== DRIVE =======================
        double throttle = .3 + (gamepad1.right_trigger * .7);
        follower.update();
        follower.setTeleOpDrive(
                (-gamepad1.left_stick_y) * throttle ,
                (-gamepad1.left_stick_x) * throttle,
                (rotation),
                true

        );

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