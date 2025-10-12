package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "front_left_drive";
        FollowerConstants.leftRearMotorName = "back_left_drive";
        FollowerConstants.rightFrontMotorName = "front_right_drive";
        FollowerConstants.rightRearMotorName = "back_right_drive";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 9.2;

        FollowerConstants.xMovement = (83.17 + 82.67 + 81.52) / 3.0;
        FollowerConstants.yMovement = (65.62 + 67.52 + 65.74) / 3.0;

        FollowerConstants.forwardZeroPowerAcceleration = (-38.04 + (-32) + (-40.17)) / 3.0;
        FollowerConstants.lateralZeroPowerAcceleration = ((-70.57) + (-66.93) + (-67.08)) / 3.0;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0.0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.05,0,0.001,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1,0,0.02,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1,0,0.01,0); // Not being used, @see useSecondaryHeadingPID


        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00035,0.6,0.0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.015,0,0.0002,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.pathEndTValueConstraint = 0.97; // was 0.95
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.maxPower = 1;

        FollowerConstants.useBrakeModeInTeleOp = true;

        // i added this back
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 12.7;
        FollowerConstants.cacheInvalidateSeconds = 0.05;
        FollowerConstants.holdPointHeadingScaling = 1;
        FollowerConstants.holdPointTranslationalScaling = 1;
    }
}
