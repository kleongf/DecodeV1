package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VoltageCompFollower extends Follower {
    public VoltageCompFollower(HardwareMap hardwareMap, Class<?> FConstants, Class<?> LConstants) {
        super(hardwareMap, FConstants, LConstants);
    }

    @Override
    public double getVoltageNormalized() {
        double frictionConstant = 0.15;
        return (FollowerConstants.nominalVoltage - (FollowerConstants.nominalVoltage * frictionConstant)) / (this.getVoltage() - ((Math.pow(FollowerConstants.nominalVoltage, 2) / this.getVoltage()) * frictionConstant));
    }
}
