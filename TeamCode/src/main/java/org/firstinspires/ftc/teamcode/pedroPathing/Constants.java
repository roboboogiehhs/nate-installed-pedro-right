package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.7913)
            .forwardZeroPowerAcceleration(-36.87845)
            .lateralZeroPowerAcceleration(-65.2857)
           /* Our Constants
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(0,0.01,0,1.2))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007,0,0.00001,0.6,0.01))
            */
            //Innovo's Constants
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1, 0, 0.00045, 0.00))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.0001,0.6,0.0))
            .centripetalScaling(0.0006);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(73.231)
            .yVelocity(39.421);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.5)
            .strafePodX(0.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}