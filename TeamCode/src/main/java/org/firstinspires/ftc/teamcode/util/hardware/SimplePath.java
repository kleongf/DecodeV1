package org.firstinspires.ftc.teamcode.util.hardware;

import com.pedropathing.localization.Pose;

public class SimplePath {
    private Pose startPose;
    private Pose endPose;
    public SimplePath(Pose startPose, Pose endPose) {
        this.startPose = startPose;
        this.endPose = endPose;
    }

    public Pose getStartPose() {return startPose;}
    public Pose getEndPose() {return endPose;}
}
