package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        // manual values
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName  = "Fl";
        FollowerConstants.leftRearMotorName   = "Bl";
        FollowerConstants.rightFrontMotorName = "Fr";
        FollowerConstants.rightRearMotorName  = "Br";

        FollowerConstants.leftFrontMotorDirection  = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection   = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection  = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13.425;

        // automatic tuner returned values
        FollowerConstants.xMovement = 60.744384045;
        FollowerConstants.yMovement = 47.778439672;

        FollowerConstants.forwardZeroPowerAcceleration = -34.3438991466;
        FollowerConstants.lateralZeroPowerAcceleration = -73.6468992726;

        // Translational PID
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.05,0,0.015,0);//0.11, 0.015
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        // Heading PID
        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.09,0); //1, 0.09
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        // Drive PID
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.013,0,0.0001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID

        // customizable values
        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.00045;

        // not in the default, jayden dug into docs to find
        FollowerConstants.motorCachingThreshold = 0.005;
        FollowerConstants.automaticHoldEnd = false; // this can be changed later

        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.02;
    }
}
