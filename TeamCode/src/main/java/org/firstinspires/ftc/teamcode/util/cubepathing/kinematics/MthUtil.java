package org.firstinspires.ftc.teamcode.util.cubepathing.kinematics;

import org.firstinspires.ftc.teamcode.util.purepursuit2.Matrix;

public class MthUtil {
    // used to convert any vector, error, pose, etc. from field frame into robot frame
    public static double[] fieldToRobot(double x, double y, double theta) {
        Matrix A = new Matrix(new double[][]{
                {Math.sin(theta), -Math.cos(theta), 0},
                {Math.cos(theta),  Math.sin(theta), 0},
                {0,     0,    1},
        });

        Matrix X = new Matrix(new double[][]{
                {x, y, theta}
        }).transpose();

        Matrix B = A.multiply(X);

        return new double[] {B.get(0, 0), B.get(1, 0), B.get(2, 0)};
    }
}
