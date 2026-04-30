package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
    // Put Robot weight in kg & 10.2 kg in pounds is 22.5 pounds
            .mass(13.1) // was 10.22
            .centripetalScaling(0.001)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(.05, .095, .0021693))
            .translationalPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.01, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.15,0,0.004,0.5,.03))

            .forwardZeroPowerAcceleration(-27.577491485309178)
            .lateralZeroPowerAcceleration(-60.75657833514421);



    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .75, .3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(69.339) // was 72
            .yVelocity(46.427) // was 55
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.4)
            // -4.4
            // 2.7
            .strafePodX(2.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
