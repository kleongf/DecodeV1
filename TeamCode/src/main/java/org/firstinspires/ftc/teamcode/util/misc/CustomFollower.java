//package org.firstinspires.ftc.teamcode.util.misc;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.pathgen.MathFunctions;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class CustomFollower extends Follower {
//    public CustomFollower(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
//        super(hardwareMap, FConstants, LConstants);
//    }
//
//    @Override
//    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
//        this.setTeleop
//        this.teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, (double)-1.0F, (double)1.0F);
//        this.teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, (double)-1.0F, (double)1.0F);
//        this.teleopDriveValues[2] = MathFunctions.clamp(heading, (double)-1.0F, (double)1.0F);
//        this.teleopDriveVector.setOrthogonalComponents(this.teleopDriveValues[0], this.teleopDriveValues[1]);
//        this.teleopDriveVector.setMagnitude(MathFunctions.clamp(this.teleopDriveVector.getMagnitude(), (double)0.0F, (double)1.0F));
//        if (robotCentric) {
//            this.teleopDriveVector.rotateVector(this.getPose().getHeading());
//        }
//
//        this.teleopHeadingVector.setComponents(this.teleopDriveValues[2], this.getPose().getHeading());
//    }
//
//    @Override
//    public double getVoltageNormalized() {
//        double frictionConstant = 0.15;
//        return (FollowerConstants.nominalVoltage -
//                (FollowerConstants.nominalVoltage * frictionConstant)) /
//                (this.getVoltage() - ((Math.pow(FollowerConstants.nominalVoltage, 2) /
//                        this.getVoltage()) * frictionConstant));
//    }
//
//}

