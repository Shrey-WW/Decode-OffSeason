package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-54.6846)
            .lateralZeroPowerAcceleration(-30.215);
//    public static OTOSConstants localizerConstants = new OTOSConstants()
//            .hardwareMapName("otos")
//            .linearUnit(DistanceUnit.INCH)
//            .angleUnit(AngleUnit.RADIANS)
//            .offset(new SparkFunOTOS.Pose2D(1.605,-2.567, -Math.PI/2))
//            .linearScalar(1.03)
//            .angularScalar(.9741);
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
        .rightFrontMotorName("fr")
        .rightRearMotorName("br")
        .leftRearMotorName("bl")
        .leftFrontMotorName("fl")
        .leftFrontEncoderDirection(Encoder.REVERSE)
        .leftRearEncoderDirection(Encoder.REVERSE)
        .rightFrontEncoderDirection(Encoder.FORWARD)
        .rightRearEncoderDirection(Encoder.FORWARD)
        .robotLength(12)
        .robotWidth(13.5)
        .forwardTicksToInches(1.1243)
        .strafeTicksToInches(.103)
        .turnTicksToInches(1.021);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78)
            .yVelocity(65);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
