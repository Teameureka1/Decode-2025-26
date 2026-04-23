package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Light")
public class Light extends LinearOpMode {

    private ColorSensor colorSensor;
    private ColorSensor sensor;



    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Intake Alpha (light)", colorSensor.alpha());
            telemetry.addData("Transfer Alpha (light)", sensor.alpha());
            telemetry.update();
        }
    }
}