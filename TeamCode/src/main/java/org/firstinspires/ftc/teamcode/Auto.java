package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
public class Auto {


    @Autonomous(name = "BlueCloseLaunchSixMove (Blocks to Java)")
    @Disabled
    public class BlueCloseLaunchSixMove extends LinearOpMode {

        private DcMotor FL;
        private DcMotor BL;
        private DcMotor FR;
        private DcMotor BR;
        private Servo blocker;
        private Servo angle;
        private DcMotor Artifactlauncher;
        private DcMotor intakewheels;
        private DcMotor intakestring;
        private Servo transfer;

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
        double FORWARD_SPEED;
        int Last_Left;
        int Last_Middle;
        int Last_Right;
        double Side_Encoder_Distance;
        int Middle_encoder_Offset;
        double Inches_per_tick;

        /**
         * This is method for moving forward and backwards
         */
        private void Slow_Forward(int Power, int Distance) {
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

        /**
         * This is method for moving forward and backwards
         */
        private void TurnRight(int Power, int Distance) {
            Foward_Ticks = FL.getCurrentPosition();
            Strafe_Ticks = -FR.getCurrentPosition();
            Backward_Ticks = BR.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            BL.setPower(FORWARD_SPEED);
            FL.setPower(FORWARD_SPEED);
            FR.setPower(-FORWARD_SPEED);
            BR.setPower(-FORWARD_SPEED);
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

        /**
         * This is method for moving forward and backwards
         */
        private void TurnLeft(int Power, int Distance) {
            Foward_Ticks = FL.getCurrentPosition();
            Strafe_Ticks = -FR.getCurrentPosition();
            Backward_Ticks = BR.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            BL.setPower(-FORWARD_SPEED);
            FL.setPower(-FORWARD_SPEED);
            FR.setPower(FORWARD_SPEED);
            BR.setPower(FORWARD_SPEED);
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

        /**
         * This is method for moving forward and backwards
         */
        private void Forward(int Power, int Distance) {
            Foward_Ticks = -FL.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FR.setPower(FORWARD_SPEED);
            FL.setPower(FORWARD_SPEED);
            BL.setPower(FORWARD_SPEED);
            BR.setPower(FORWARD_SPEED);
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

        /**
         * This is method for moving forward and backwards
         */
        private void Backward(int Power, int Distance) {
            Foward_Ticks = FL.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FR.setPower(-FORWARD_SPEED);
            FL.setPower(-FORWARD_SPEED);
            BL.setPower(-FORWARD_SPEED);
            BR.setPower(-FORWARD_SPEED);
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

        /**
         * Describe this function...
         */
        @Override
        public void runOpMode() {
            FL = hardwareMap.get(DcMotor.class, "FLAsDcMotor");
            BL = hardwareMap.get(DcMotor.class, "BLAsDcMotor");
            FR = hardwareMap.get(DcMotor.class, "FRAsDcMotor");
            BR = hardwareMap.get(DcMotor.class, "BRAsDcMotor");
            blocker = hardwareMap.get(Servo.class, "blockerAsServo");
            angle = hardwareMap.get(Servo.class, "angle");
            Artifactlauncher = hardwareMap.get(DcMotor.class, "ArtifactlauncherAsDcMotor");
            intakewheels = hardwareMap.get(DcMotor.class, "intake wheels");
            intakestring = hardwareMap.get(DcMotor.class, "intake string");
            transfer = hardwareMap.get(Servo.class, "transferAsServo");

            InitOdometry();
            waitForStart();
            if (opModeIsActive()) {
                blocker.setPosition(0.94);
                angle.setPosition(0.6);
                ((DcMotorEx) Artifactlauncher).setVelocity(1300);
                Reset_Encoders();
                telemetry.addData("Velocity", ((DcMotorEx) Artifactlauncher).getVelocity());
                telemetry.addData("x", Get_X());
                telemetry.addData("y", Get_Y());
                telemetry.addData("z", Get_Z());
                telemetry.update();
                Reset_Encoders();
                Forward(1, 11000);
                Reset_Encoders();
                Launch_Three();
                Reset_Encoders();
                sleep(1000);
                Artifactlauncher.setPower(0);
                blocker.setPosition(0.86);
                Reset_Encoders();
                TurnRight(1, 2450);
                Reset_Encoders();
                StrafeLeft(1, 4850);
                intakewheels.setPower(-1);
                intakestring.setPower(-1);
                sleep(1000);
                Slow_Forward(1, 11300);
                Reset_Encoders();
                Backward(1, 6500);
                Reset_Encoders();
                sleep(250);
                intakewheels.setPower(1);
                intakestring.setPower(-1);
                sleep(1000);
                intakestring.setPower(0);
                intakewheels.setPower(0);
                Reset_Encoders();
                TurnLeft(1, 3100);
                Reset_Encoders();
                StrafeLeft(1, 2950);
                Reset_Encoders();
                blocker.setPosition(0.94);
                Reset_Encoders();
                Launch_Three();
                Reset_Encoders();
                Artifactlauncher.setPower(0);
                blocker.setPosition(0.86);
                sleep(800);
                Reset_Encoders();
                Backward(1, 5000);
                StrafeLeft(1, 5000);
            }
        }

        /**
         * This is method for moving forward and backwards
         */
        private void Launch(int Power, int Distance) {
            ((DcMotorEx) Artifactlauncher).setVelocity(1345);
            sleep(900);
            transfer.setPosition(0.55);
            sleep(400);
            transfer.setPosition(0.22);
        }

        /**
         * This is method for moving forward and backwards
         */
        private void StrafeLeft(int Power, int Distance) {
            Backward_Ticks = BR.getCurrentPosition();
            Foward_Ticks = -FL.getCurrentPosition();
            Strafe_Ticks = FR.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FR.setPower(FORWARD_SPEED);
            FL.setPower(-FORWARD_SPEED);
            BL.setPower(FORWARD_SPEED);
            BR.setPower(-FORWARD_SPEED);
            while (opModeIsActive() && Strafe_Ticks < Distance) {
                if (FL.getCurrentPosition() > 0) {
                    FL.setPower(-FORWARD_SPEED + 0.027);
                    BL.setPower(FORWARD_SPEED + 0.027);
                    BR.setPower(-FORWARD_SPEED);
                    FR.setPower(FORWARD_SPEED);
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

        /**
         * This is method for moving forward and backwards
         */
        private void StrafeRight(double Distance) {
            Backward_Ticks = BR.getCurrentPosition();
            Foward_Ticks = -FL.getCurrentPosition();
            Strafe_Ticks = -FR.getCurrentPosition();
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);
            FR.setPower(-FORWARD_SPEED);
            FL.setPower(FORWARD_SPEED);
            BL.setPower(-FORWARD_SPEED);
            BR.setPower(FORWARD_SPEED);
            while (opModeIsActive() && Strafe_Ticks < Distance) {
                if (FL.getCurrentPosition() > 0) {
                    FL.setPower(FORWARD_SPEED - 0.027);
                    BL.setPower(-FORWARD_SPEED - 0.027);
                    BR.setPower(FORWARD_SPEED);
                    FR.setPower(-FORWARD_SPEED);
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

        /**
         * This is method for moving forward and backwards
         */
        private void Launch2(int Power, int Distance) {
            ((DcMotorEx) Artifactlauncher).setVelocity(1340);
            sleep(800);
            transfer.setPosition(0.55);
            sleep(400);
            transfer.setPosition(0.22);
        }

        /**
         * Describe this function...
         */
        private void Launch_Three2() {
            Launch2(1500, 700);
            sleep(125);
            intakestring.setPower(-1);
            sleep(700);
            intakestring.setPower(0);
            Launch2(100, 100);
            sleep(125);
            intakestring.setPower(-1);
            sleep(800);
            intakestring.setPower(0);
            Launch2(1, 100);
            ((DcMotorEx) Artifactlauncher).setVelocity(1380);
        }

        /**
         * Describe this function...
         */
        private void Launch_Three() {
            Launch(1500, 700);
            sleep(125);
            intakestring.setPower(-1);
            sleep(700);
            intakestring.setPower(0);
            Launch(100, 100);
            sleep(125);
            intakestring.setPower(-1);
            sleep(800);
            intakestring.setPower(0);
            Launch(1, 100);
            ((DcMotorEx) Artifactlauncher).setVelocity(1400);
        }

        /**
         * Describe this function...
         */
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

            if (FlipLeft) {
                Raw_Left_Position = BL.getCurrentPosition() * -1;
            } else {
                Raw_Left_Position = BL.getCurrentPosition() * 1;
            }
            if (FlipMiddle) {
                Raw_Middle_Position = FR.getCurrentPosition() * -1;
            } else {
                Raw_Middle_Position = FR.getCurrentPosition() * 1;
            }
            if (FlipRight) {
                Raw_Right_Position = BR.getCurrentPosition() * -1;
            } else {
                Raw_Right_Position = BR.getCurrentPosition() * 1;
            }
            Left_Position = Raw_Left_Position - Last_Left;
            Middle_Position = Raw_Middle_Position - Last_Middle;
            Right_Position = Raw_Right_Position - Last_Right;
            Last_Left = Raw_Left_Position;
            Last_Middle = Raw_Middle_Position;
            Last_Right = Raw_Right_Position;
            Initial_Z = Inches_per_tick * ((Right_Position - Left_Position) / Side_Encoder_Distance);
            Initial_Y = Inches_per_tick * ((Left_Position + Right_Position) / 2);
            Initial_X = Inches_per_tick * (Middle_Position - (Right_Position - Left_Position)) * (Middle_encoder_Offset / Side_Encoder_Distance);
            Z = (int) (Z - Initial_Z);
            Theta = Z + Initial_Z / 2;
            Relative_Y = Initial_Y * (Math.cos((Theta * (180 / Math.PI)) / 180 * Math.PI) - Initial_X * Math.sin(Theta / 180 * Math.PI));
            Relative_X = Initial_Y * (Math.sin(Theta / 180 * Math.PI) + Initial_X * Math.cos(Theta / 180 * Math.PI));
            X = (int) (X + Relative_X);
            Y = (int) (Y + Relative_Y);
        }

        /**
         * Describe this function...
         */
        private void Reset_To(double coolx, double cooly, double coolz) {
            X_offset = (int) (Get_X() + (X_offset - coolx));
            Y_offset = (int) (Get_Y() + (Y_offset - cooly));
            Z_Offset = (int) (Get_Z() + (Z_Offset - coolz));
        }

        /**
         * Describe this function...
         */
        private void InitOdometry() {
            int Distance;
            double Encoder_Wheel_Radius;
            int Encoder_Ticks_per_Rotation;

            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Distance = 200;
            FORWARD_SPEED = 0.65;
            Foward_Ticks = 0.5;
            FlipLeft = false;
            FlipMiddle = false;
            FlipRight = true;
            Side_Encoder_Distance = 9.375;
            Middle_encoder_Offset = 4;
            Encoder_Wheel_Radius = 0.944882;
            Encoder_Ticks_per_Rotation = 2000;
            Inches_per_tick = 2 * Math.PI * (Encoder_Wheel_Radius / Encoder_Ticks_per_Rotation);
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

        /**
         * Describe this function...
         */
        private double Get_X() {
            Odometry_Update();
            return X;
        }

        /**
         * Describe this function...
         */
        private double Get_Y() {
            Odometry_Update();
            return Y;
        }

        /**
         * Describe this function...
         */
        private double Get_Z() {
            Odometry_Update();
            return Z;
        }
    }

}
