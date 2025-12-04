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
    public static double linearScalar = (1.026483699897225 + 1.0126931711026617 + 1.0183723069079784) / 3;
    public static double angularScalar = (1.0369284199286806 + 1.0316728273289406) / 2;

    // offset values
    public static double x_offset;
    public static double y_offset;


    // Follower Constants
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration((-48.797532039760966 + -48.08819084142787 + -49.617025777709834) / 3)
            .lateralZeroPowerAcceleration((-69.1192435003088 + -72.3474983053327 + -64.24053799899083) / 3)
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
            .xVelocity((69.35059554933562 + 63.18280077356053 + 65.66505732498769) / 3)
            .yVelocity((52.194670429379926 + 52.0847350593627 + 51.52544637364665) / 3);

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