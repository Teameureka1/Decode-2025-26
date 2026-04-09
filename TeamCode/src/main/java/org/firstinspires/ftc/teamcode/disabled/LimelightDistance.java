package org.firstinspires.ftc.teamcode.disabled;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class LimelightDistance extends OpMode {

    private Limelight3A limelight3A;


    private double Camera_Height_CM = 25.85;
    private double Camera_Angle = 19;
    private double Goal_Height = 74.95;
    private double distance = 0;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
        limelight3A.start();
    }


    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();


        if (llResult != null && llResult.isValid()) {
            distance = getDistance(llResult.getTy());
            telemetry.addData("Distance", distance);
        } else {
            telemetry.addData("No Target", "Found");
        }

    }

    public double getDistance(double ty) {
         double angleToTarget = Camera_Angle + ty;
               double heightDifference = Goal_Height - Camera_Height_CM ;
               return heightDifference / Math.tan(Math.toRadians(angleToTarget));

    }

}
