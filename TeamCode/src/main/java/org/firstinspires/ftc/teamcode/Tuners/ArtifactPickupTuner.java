package org.firstinspires.ftc.teamcode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Config;

@TeleOp
public class ArtifactPickupTuner extends OpMode {

    private Config robot;

    double intakeSpeed = 0;
    double speed = 0;

    double[] stepSizes = {1000.00, 100.00, 10.0, 1.0, 0.1, .7, .001, .001};

    int stepIndex = 1;


    @Override
    public void init() {
        robot = new Config(this);
        robot.init();
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {



        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            intakeSpeed += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            intakeSpeed -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            speed += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            speed -= stepSizes[stepIndex];
        }
        robot.intake.setVelocity(intakeSpeed);
        robot.kicker.setVelocity(intakeSpeed);
        robot.frontLeftMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);

        telemetry.addLine("------------------------------------------------");
        telemetry.addData("Robot Speed", "%.2f", speed);
        telemetry.addData("Intake Velocity", "%.2f", intakeSpeed);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
