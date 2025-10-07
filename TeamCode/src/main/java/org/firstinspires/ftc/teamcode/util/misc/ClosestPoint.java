package org.firstinspires.ftc.teamcode.util.misc;

import com.pedropathing.localization.Pose;

import java.util.ArrayList;

public class ClosestPoint {
    private ArrayList<Pose> poses = new ArrayList<>();
    public ClosestPoint() {
        for (int i = 24; i < 72; i++) {
            poses.add(new Pose(i, 144-i, Math.toRadians(0)));
        }
        for (int i = 72; i < 120; i++) {
            poses.add(new Pose(i, i, Math.toRadians(0)));
        }
    }

    public Pose closestPose(Pose robotPose) {
        ArrayList<Double> distances = new ArrayList<>();
        for (Pose p: poses) {
            double dist = Math.hypot(robotPose.getX()-p.getX(), robotPose.getY()-p.getY());
            distances.add(dist);
        }

        int minIndex = 0;
        double minValue = distances.get(0);

        for (int i = 1; i < distances.size(); i++) {
            if (distances.get(i) < minValue) {
                minValue = distances.get(i);
                minIndex = i;
            }
        }

        System.out.println("MIN X: " + poses.get(minIndex).getX());
        System.out.println("MIN Y: " + poses.get(minIndex).getY());

        return poses.get(minIndex);
    }
}
