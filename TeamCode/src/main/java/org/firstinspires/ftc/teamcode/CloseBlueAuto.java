package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


    @Autonomous(name = "CloseBlueAuto")
    public class CloseBlueAuto extends LinearOpMode {

        private DcMotor fl;
        private DcMotor bl;
        private DcMotor fr;
        private DcMotor br
                ;
        private Servo wall;
        private Servo angle;
        private DcMotor launcher;
        private DcMotor intakewheels;
        private DcMotor intakestring;
        private Servo kicker;

        double Foward_Ticks;
        int Backward_Ticks;
        int X_offset;
        int X;
        int Y;
        int Z;
        int Strafe_Ticks;
        boolean flipLeft;
        int Y_offset;
        boolean flipMiddle;
        int Z_Offset;
        boolean flipRight;
        double drivePower;
        int Last_Left;
        int Last_Middle;
        int Last_Right;
        double Side_Encoder_Distance;
        int Middle_encoder_Offset;
        double Inches_per_tick;

        private void Slow_Forward(int Power, int Distance) {
            Foward_Ticks = -fl.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setPower(0.5);
            fl.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(0.5);
            while (opModeIsActive() && Foward_Ticks < Distance) {
                telemetry.addData("Motor ", fl.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Foward_Ticks = -fl.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        private void TurnRight(int Power, int Distance) {
            Foward_Ticks = fl.getCurrentPosition();
            Strafe_Ticks = -fr.getCurrentPosition();
            Backward_Ticks = br.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            bl.setPower(drivePower);
            fl.setPower(drivePower);
            fr.setPower(-drivePower);
            br.setPower(-drivePower);
            while (opModeIsActive() && Strafe_Ticks < Distance && Foward_Ticks < Distance) {
                telemetry.addData("Motor fl", -fl.getCurrentPosition());
                telemetry.addData("Motor fr", fr.getCurrentPosition());
                telemetry.addData("Motor br", br.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Strafe_Ticks = fr.getCurrentPosition();
                Backward_Ticks = br.getCurrentPosition();
                Foward_Ticks = fl.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }


        private void TurnLeft(int Power, int Distance) {
            Foward_Ticks = fl.getCurrentPosition();
            Strafe_Ticks = -fr.getCurrentPosition();
            Backward_Ticks = br
                    .getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            bl.setPower(-drivePower);
            fl.setPower(-drivePower);
            fr.setPower(drivePower);
            br.setPower(drivePower);
            while (opModeIsActive() && Strafe_Ticks < Distance && Foward_Ticks < Distance) {
                telemetry.addData("Motor fl", -fl.getCurrentPosition());
                telemetry.addData("Motor fr", fr.getCurrentPosition());
                telemetry.addData("Motor br" + "", br.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Strafe_Ticks = fr.getCurrentPosition();
                Backward_Ticks = br.getCurrentPosition();
                Foward_Ticks = fl.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        /**
         * This is method for moving forward and backwards
         */
        private void Forward(int Power, int Distance) {
            Foward_Ticks = -fl.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setPower(drivePower);
            fl.setPower(drivePower);
            bl.setPower(drivePower);
            br.setPower(drivePower);
            while (opModeIsActive() && Foward_Ticks < Distance) {
                telemetry.addData("Motor ", fl.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Foward_Ticks = -fl.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        private void Backward(int Power, int Distance) {
            Foward_Ticks = fl.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setPower(-drivePower);
            fl.setPower(-drivePower);
            bl.setPower(-drivePower);
            br.setPower(-drivePower);
            while (opModeIsActive() && Foward_Ticks < Distance) {
                telemetry.addData("Motor ", fl.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Foward_Ticks = fl.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        @Override
        public void runOpMode() {
            fl = hardwareMap.get(DcMotor.class, "fl");
            bl = hardwareMap.get(DcMotor.class, "bl");
            fr = hardwareMap.get(DcMotor.class, "fr");
            br = hardwareMap.get(DcMotor.class, "br");
            wall = hardwareMap.get(Servo.class, "wall");
            angle = hardwareMap.get(Servo.class, "angle");
            launcher = hardwareMap.get(DcMotor.class, "launcher");
            intakewheels = hardwareMap.get(DcMotor.class, "intake wheels");
            intakestring = hardwareMap.get(DcMotor.class, "intake string");
            kicker = hardwareMap.get(Servo.class, "kicker");

            InitOdometry();
            waitForStart();
            if (opModeIsActive()) {
                wall.setPosition(0.94);
                angle.setPosition(.61);
                ((DcMotorEx) launcher).setVelocity(1300);
                Reset_Encoders();
                telemetry.addData("Velocity", ((DcMotorEx) launcher).getVelocity());
                telemetry.addData("x", Get_X());
                telemetry.addData("y", Get_Y());
                telemetry.addData("z", Get_Z());
                telemetry.update();
                Reset_Encoders();
                Forward(1, 12000);
                Reset_Encoders();
                Launch_Three();
                Reset_Encoders();
                sleep(1000);
                launcher.setPower(0);
                wall.setPosition(0.86);
                Reset_Encoders();
                TurnRight(1, 2800);
                Reset_Encoders();
                StrafeLeft(1, 4600);
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
                wall.setPosition(0.94);
                Reset_Encoders();
                Launch_Three();
                Reset_Encoders();
                launcher.setPower(0);
                wall.setPosition(0.86);
                sleep(800);
                Reset_Encoders();
                Backward(1, 5000);
                StrafeLeft(1, 5000);
            }
        }

        private void Launch(int Power, int Distance) {
            ((DcMotorEx) launcher).setVelocity(1400);
            sleep(500);
            kicker.setPosition(0.55);
            sleep(300);
            kicker.setPosition(0.22);
        }

        private void StrafeLeft(int Power, int Distance) {
            Backward_Ticks = br.getCurrentPosition();
            Foward_Ticks = -fl.getCurrentPosition();
            Strafe_Ticks = fr.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setPower(drivePower);
            fl.setPower(-drivePower);
            bl.setPower(drivePower);
            br.setPower(-drivePower);
            while (opModeIsActive() && Strafe_Ticks < Distance) {
                if (fl.getCurrentPosition() > 0) {
                    fl.setPower(-drivePower + 0.027);
                    bl.setPower(drivePower + 0.027);
                    br.setPower(-drivePower);
                    fr.setPower(drivePower);
                    telemetry.addData("Motor fl ", fl.getCurrentPosition());
                    telemetry.addData("Motor fr", -fr.getCurrentPosition());
                    telemetry.addData("Motor br" + "", br.getCurrentPosition());
                    telemetry.addData("Ticks", Distance);
                    Backward_Ticks = br.getCurrentPosition();
                    Foward_Ticks = fl.getCurrentPosition();
                    Strafe_Ticks = -fr.getCurrentPosition();
                    telemetry.update();
                }
                telemetry.addData("Motor fl ", fl.getCurrentPosition());
                telemetry.addData("Motor fr", -fr.getCurrentPosition());
                telemetry.addData("Motor br" + "", br.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Backward_Ticks = br.getCurrentPosition();
                Foward_Ticks = fl.getCurrentPosition();
                Strafe_Ticks = -fr.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        private void setDrivePower(double percent) {

            // Limit from 10% to 100%
            if (percent < 0.1) percent = 0.1;
            if (percent > 1.0) percent = 1.0;

            drivePower = percent;
        }

        private void StrafeRight(double Distance) {
            Backward_Ticks = br.getCurrentPosition();
            Foward_Ticks = -fl.getCurrentPosition();
            Strafe_Ticks = -fr.getCurrentPosition();
            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fr.setPower(-drivePower);
            fl.setPower(drivePower);
            bl.setPower(-drivePower);
            br.setPower(drivePower);
            while (opModeIsActive() && Strafe_Ticks < Distance) {
                if (fl.getCurrentPosition() > 0) {
                    fl.setPower(drivePower - 0.027);
                    bl.setPower(-drivePower - 0.027);
                    br.setPower(drivePower);
                    fr.setPower(-drivePower);
                    telemetry.addData("Motor fl ", fl.getCurrentPosition());
                    telemetry.addData("Motor fr", -fr.getCurrentPosition());
                    telemetry.addData("Motor br" + "", br.getCurrentPosition());
                    telemetry.addData("Ticks", Distance);
                    Backward_Ticks = br.getCurrentPosition();
                    Foward_Ticks = fl.getCurrentPosition();
                    Strafe_Ticks = -fr.getCurrentPosition();
                    telemetry.update();
                }
                telemetry.addData("Motor fl ", fl.getCurrentPosition());
                telemetry.addData("Motor fr", fr.getCurrentPosition());
                telemetry.addData("Motor br" +
                        "", br.getCurrentPosition());
                telemetry.addData("Ticks", Distance);
                Backward_Ticks = br.getCurrentPosition();
                Foward_Ticks = fl.getCurrentPosition();
                Strafe_Ticks = fr.getCurrentPosition();
                telemetry.update();
            }
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        private void Launch2(int Power, int Distance) {
            ((DcMotorEx) launcher).setVelocity(1340);
            sleep(800);
            kicker.setPosition(0.55);
            sleep(400);
            kicker.setPosition(0.22);
        }

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
            ((DcMotorEx) launcher).setVelocity(1380);
        }

        private void Launch_Three() {
            Launch(1500, 700);
            sleep(125);
            intakestring.setPower(-1);
            sleep(600);
            intakestring.setPower(0);
            Launch(100, 100);
            sleep(125);
            intakestring.setPower(-1);
            sleep(600);
            intakestring.setPower(0);
            Launch(1, 100);
            ((DcMotorEx) launcher).setVelocity(1400);
        }

        private void Reset_Encoders() {
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            if (flipLeft) {
                Raw_Left_Position = bl.getCurrentPosition() * -1;
            } else {
                Raw_Left_Position = bl.getCurrentPosition() * 1;
            }
            if (flipMiddle) {
                Raw_Middle_Position = fr.getCurrentPosition() * -1;
            } else {
                Raw_Middle_Position = fr.getCurrentPosition() * 1;
            }
            if (flipRight) {
                Raw_Right_Position = br.getCurrentPosition() * -1;
            } else {
                Raw_Right_Position = br.getCurrentPosition() * 1;
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

            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Distance = 200;
            drivePower = 0.65;
            Foward_Ticks = 0.5;
            flipLeft = false;
            flipMiddle = false;
            flipRight = true;
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
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
