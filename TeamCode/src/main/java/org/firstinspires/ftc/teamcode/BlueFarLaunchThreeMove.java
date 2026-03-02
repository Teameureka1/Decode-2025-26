
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
  import com.qualcomm.robotcore.hardware.Servo;

  @Autonomous(name = "BlueFarLaunchThreeMove (Blocks to Java)")
  public class BlueFarLaunchThreeMove extends LinearOpMode {

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo blocker;
    private Servo angle;
    private DcMotor Artifactlauncher;
    private Servo transfer;
    private DcMotor intakestring;

    double Foward_Ticks;
    int Backward_Ticks;
    int X_offset;
    int X;
    int Y;
    int Z;
    int Strafe_Ticks;
    boolean FlipLeft;
    int Y_offset;
    boolean FlipMiddle;
    int Z_Offset;
    boolean FlipRight;
    double Power;
    int Last_Left;
    int Last_Middle;
    int Last_Right;
   double Side_Encoder_Distance;
    int Middle_encoder_Offset;
    double Inches_per_tick;
 private void Slow_Forward(double Power, double Distance) {
 Foward_Ticks = -FL.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 FR.setPower(0.5);
 FL.setPower(0.5);
 BL.setPower(0.5);
 BR.setPower(0.5);
 while (opModeIsActive() && Foward_Ticks < Distance) {
 telemetry.addData("Motor ", FL.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Foward_Ticks = -FL.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }


private void TurnRight(int Power, double Distance) {
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 Backward_Ticks = BR.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 BL.setPower(BlueFarLaunchThreeMove.this.Power);
 FL.setPower(BlueFarLaunchThreeMove.this.Power);
 FR.setPower(-BlueFarLaunchThreeMove.this.Power);
 BR.setPower(-BlueFarLaunchThreeMove.this.Power);
 while (opModeIsActive() && Strafe_Ticks < Distance && Foward_Ticks < Distance) {
 telemetry.addData("Motor FL", -FL.getCurrentPosition());
 telemetry.addData("Motor FR", FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Strafe_Ticks = FR.getCurrentPosition();
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }

private void TurnLeft(int Power, int Distance) {
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 Backward_Ticks = BR.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 BL.setPower(-BlueFarLaunchThreeMove.this.Power);
 FL.setPower(-BlueFarLaunchThreeMove.this.Power);
 FR.setPower(BlueFarLaunchThreeMove.this.Power);
 BR.setPower(BlueFarLaunchThreeMove.this.Power);
 while (opModeIsActive() && Strafe_Ticks < Distance && Foward_Ticks < Distance) {
 telemetry.addData("Motor FL", -FL.getCurrentPosition());
 telemetry.addData("Motor FR", FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Strafe_Ticks = FR.getCurrentPosition();
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }



private void Forward(int Power, double Distance) {
 Foward_Ticks = -FL.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 FR.setPower(Power);
 FL.setPower(Power);
 BL.setPower(Power);
 BR.setPower(Power);
 while (opModeIsActive() && Foward_Ticks < Distance) {
 telemetry.addData("Motor ", FL.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Foward_Ticks = -FL.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }

private void Backward(int Power, int Distance) {
 Foward_Ticks = FL.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 FR.setPower(-BlueFarLaunchThreeMove.this.Power);
 FL.setPower(-BlueFarLaunchThreeMove.this.Power);
 BL.setPower(-BlueFarLaunchThreeMove.this.Power);
 BR.setPower(-BlueFarLaunchThreeMove.this.Power);
 while (opModeIsActive() && Foward_Ticks < Distance) {
 telemetry.addData("Motor ", FL.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Foward_Ticks = FL.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }

 @Override
public void runOpMode() {
 FL = hardwareMap.get(DcMotor.class, "FLAsDcMotor");
 BL = hardwareMap.get(DcMotor.class, "BLAsDcMotor");
 FR = hardwareMap.get(DcMotor.class, "FRAsDcMotor");
 BR = hardwareMap.get(DcMotor.class, "BRAsDcMotor");
 blocker = hardwareMap.get(Servo.class, "blockerAsServo");
 angle = hardwareMap.get(Servo.class, "angle");
 Artifactlauncher = hardwareMap.get(DcMotor.class, "ArtifactlauncherAsDcMotor");
 transfer = hardwareMap.get(Servo.class, "transferAsServo");
 intakestring = hardwareMap.get(DcMotor.class, "intake string");

 InitOdometry();
 waitForStart();
 if (opModeIsActive()) {
 blocker.setPosition(0.94);
 angle.setPosition(0.62);
 ((DcMotorEx) Artifactlauncher).setVelocity(2500);
 Reset_Encoders();
 telemetry.addData("Velocity", ((DcMotorEx) Artifactlauncher).getVelocity());
 telemetry.addData("x", Get_X());
 telemetry.addData("y", Get_Y());
 telemetry.addData("z", Get_Z());
 telemetry.update();
 Reset_Encoders();
 Backward(1, 200);
 Reset_Encoders();
 sleep(1);
 Reset_Encoders();
 TurnLeft(1, 735);
 Reset_Encoders();
 Launch_Three();
 sleep(1000);
 Artifactlauncher.setPower(0);
 blocker.setPosition(0.86);
 Reset_Encoders();
 StrafeRight(1, 6000);
 }
 }



private void Launch(int Power, int Distance) {
 ((DcMotorEx) Artifactlauncher).setVelocity(1685);
 sleep(625);
 transfer.setPosition(0.55);
 sleep(400);
 transfer.setPosition(0.22);
 }

private void StrafeLeft(double Power, double Distance) {
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = -FL.getCurrentPosition();
 Strafe_Ticks = FR.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 FR.setPower(BlueFarLaunchThreeMove.this.Power);
 FL.setPower(-BlueFarLaunchThreeMove.this.Power);
 BL.setPower(BlueFarLaunchThreeMove.this.Power);
 BR.setPower(-BlueFarLaunchThreeMove.this.Power);
 while (opModeIsActive() && Strafe_Ticks < Distance) {
 if (FL.getCurrentPosition() > 0) {
 FL.setPower(-BlueFarLaunchThreeMove.this.Power + 0.027);
 BL.setPower(BlueFarLaunchThreeMove.this.Power + 0.027);
 BR.setPower(-BlueFarLaunchThreeMove.this.Power);
 FR.setPower(BlueFarLaunchThreeMove.this.Power);
 telemetry.addData("Motor FL ", FL.getCurrentPosition());
 telemetry.addData("Motor FR", -FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 telemetry.update();
 }
 telemetry.addData("Motor FL ", FL.getCurrentPosition());
 telemetry.addData("Motor FR", -FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }


private void StrafeRight(int Power, int Distance) {
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = -FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 FL.setDirection(DcMotor.Direction.REVERSE);
 BL.setDirection(DcMotor.Direction.REVERSE);
 FR.setPower(-BlueFarLaunchThreeMove.this.Power);
 FL.setPower(BlueFarLaunchThreeMove.this.Power);
 BL.setPower(-BlueFarLaunchThreeMove.this.Power);
 BR.setPower(BlueFarLaunchThreeMove.this.Power);
 while (opModeIsActive() && Strafe_Ticks < Distance) {
 if (FL.getCurrentPosition() > 0) {
 FL.setPower(BlueFarLaunchThreeMove.this.Power - 0.027);
 BL.setPower(-BlueFarLaunchThreeMove.this.Power - 0.027);
 BR.setPower(BlueFarLaunchThreeMove.this.Power);
 FR.setPower(-BlueFarLaunchThreeMove.this.Power);
 telemetry.addData("Motor FL ", FL.getCurrentPosition());
 telemetry.addData("Motor FR", -FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = -FR.getCurrentPosition();
 telemetry.update();
 }
 telemetry.addData("Motor FL ", FL.getCurrentPosition());
 telemetry.addData("Motor FR", FR.getCurrentPosition());
 telemetry.addData("Motor BR", BR.getCurrentPosition());
 telemetry.addData("Ticks", Distance);
 Backward_Ticks = BR.getCurrentPosition();
 Foward_Ticks = FL.getCurrentPosition();
 Strafe_Ticks = FR.getCurrentPosition();
 telemetry.update();
 }
 FR.setPower(0);
 FL.setPower(0);
 BL.setPower(0);
 BR.setPower(0);
 }


private void Launch2(int Power, int Distance) {
 ((DcMotorEx) Artifactlauncher).setVelocity(1720);
 sleep(625);
 transfer.setPosition(0.55);
 sleep(400);
 transfer.setPosition(0.22);
 }


private void Launch_Three2() {
 sleep(700);
 Launch2(1500, 700);
 sleep(125);
 intakestring.setPower(-1);
 sleep(800);
 intakestring.setPower(0);
 Launch2(100, 100);
 sleep(125);
 intakestring.setPower(-1);
 sleep(1000);
 intakestring.setPower(0);
 Launch2(1, 100);
 ((DcMotorEx) Artifactlauncher).setVelocity(1730);
 }



private void Launch_Three() {
 sleep(800);
 Launch(1500, 700);
 sleep(125);
 intakestring.setPower(-1);
 sleep(800);
 intakestring.setPower(0);
 Launch(100, 100);
 sleep(125);
 intakestring.setPower(-1);
 sleep(1000);
 intakestring.setPower(0);
 Launch(1, 100);
 ((DcMotorEx) Artifactlauncher).setVelocity(1730);
 }


private void Reset_Encoders() {
 BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 }


private void Odometry_Update() {
 int Raw_Left_Position;
 int Raw_Middle_Position;
 int Raw_Right_Position;
 double Left_Position;
 double Middle_Position;
 double Right_Position;
 double Initial_Z;
 double Initial_Y;
 double Initial_X;
 double Theta;
 double Relative_Y;
 double Relative_X;

    int i = 1;
    if (FlipLeft) {
 Raw_Left_Position = BL.getCurrentPosition()  - i;
 } else {
 Raw_Left_Position = BL.getCurrentPosition()  i;
 }
 if (FlipMiddle) {
 Raw_Middle_Position = FR.getCurrentPosition()  - i;
 } else {
 Raw_Middle_Position = FR.getCurrentPosition()  i;
 }
 if (FlipRight) {
 Raw_Right_Position = BR.getCurrentPosition()  - i;
 } else {
 Raw_Right_Position = BR.getCurrentPosition()  i;
 }
 Left_Position = Raw_Left_Position - Last_Left;
 Middle_Position = Raw_Middle_Position - Last_Middle;
 Right_Position = Raw_Right_Position - Last_Right;
 Last_Left = Raw_Left_Position;
 Last_Middle = Raw_Middle_Position;
 Last_Right = Raw_Right_Position;
 Initial_Z = Inches_per_tick  ((Right_Position - Left_Position) / Side_Encoder_Distance);
    Initial_X = Inches_per_tick  (Middle_Position - (Right_Position - Left_Position))  (Middle_encoder_Offset / Side_Encoder_Distance);
 Z = (int) (Z - Initial_Z);
 Theta = Z + Initial_Z / 2;

    Relative_Y = Initial_Y  (Math.cos((Theta  (180 / Math.PI)) / 180  Math.PI) - Initial_X  Math.sin(Theta / 180  Math.PI));

    Relative_X = Initial_Y  (Math.sin(Theta / 180  Math.PI) + Initial_X  Math.cos(Theta / 180  Math.PI));
 X = (int) (X + Relative_X);
 Y = (int) (Y + Relative_Y);
 }



private void Reset_To(double coolx, double cooly, double coolz) {
 X_offset = (int) (Get_X() + (X_offset - coolx));
 Y_offset = (int) (Get_Y() + (Y_offset - cooly));
 Z_Offset = (int) (Get_Z() + (Z_Offset - coolz));
 }

private void InitOdometry() {
 int Distance;
 double Encoder_Wheel_Radius;
 int Encoder_Ticks_per_Rotation;

 FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 Distance = 200;
 Power = 0.65;
 Foward_Ticks = 0.5;
 FlipLeft = false;
 FlipMiddle = false;
 FlipRight = true;
 Side_Encoder_Distance = 9.375;
 Middle_encoder_Offset = 4;
 Encoder_Wheel_Radius = 0.944882;
 Encoder_Ticks_per_Rotation = 2000;
 Inches_per_tick = 2  Math.PI  (Encoder_Wheel_Radius / Encoder_Ticks_per_Rotation);
 X = 0;
 Y = 0;
 Z = 0;
 X_offset = 0;
 Y_offset = 0;
 Z_Offset = 0;
 Last_Left = 0;
 Last_Middle = 0;
 Last_Right = 0;
 BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
 }

private double Get_X() {
 Odometry_Update();
 return X;
 }

private double Get_Y() {
 Odometry_Update();
 return Y;
 }


private double Get_Z() {
 Odometry_Update();
 return Z;
 }
 }

@TeleOp

public class awesometele extends LinearOpMode {
    ThreeWheeled odometer;


    @Override
    public void runOpMode() {

        odometer = new ThreeWheeled.Builder()
                .setLeftEncoder(bl)
                .setRightEncoder(fr)
                .setMiddleEncoder(br)

                .setEncoderTicksPerRotation(2000)
                .setEncoderWheelRadius(0.944882)

                //Change the true/false values to correct directions
                .setFlipLeftEncoder(false)
                .setFlipRightEncoder(true)
                .setFlipMiddleEncoder(false)

                .setSideEncoderDistance(9.375)
                .setMiddleEncoderOffset(4)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
