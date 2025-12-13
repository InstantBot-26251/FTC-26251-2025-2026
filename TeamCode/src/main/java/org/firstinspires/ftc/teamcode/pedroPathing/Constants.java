package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    // tuning values
    public static double linearScalar =  (1.0552050224572318 + 1.0552050224572318) / 2;
    public static double angularScalar = (1.0870443195300399 + 1.066260077695128) / 2;

    // offset values
    public static double x_offset;
    public static double y_offset;


    // Follower Constants
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.888314)
            .forwardZeroPowerAcceleration((-51.6758158363617 + -53.427311796983965 + -48.728190211534475) / 3)
            .lateralZeroPowerAcceleration((-99.76900543611134 + -86.39340308348679 + -90.30358998760529) / 3)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.3, 0.0, 0.01, 0.6, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.28, 0.0,0.021,0.015))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.029,0.0, 0.001, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5,0.0, 0.0015, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1,0.0, 0.08, 0.01))
            .centripetalScaling(0.0005)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true);

    // Path Constraints
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // Mecanum Constants
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity((47.347301573265256 + 56.77290788785679 + 56.51579128475639) / 3)
            .yVelocity((44.45654200756643 + 44.09850113035188) / 2);

    // OTOS Constants
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .offset(new SparkFunOTOS.Pose2D(x_offset, y_offset, (3*Math.PI)/2)) // sets offset
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            // OTOS Tuning Values
            .linearScalar(linearScalar)
            .angularScalar(angularScalar);


    // Create Follower Method
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}