package org.firstinspires.ftc.teamcode.util.cubepathing.kinematics;

import org.firstinspires.ftc.teamcode.util.purepursuit2.Matrix;

public class MecanumDriveKinematics {
    private double lengthX;
    private double lengthY;
    private Matrix TInv;
    public MecanumDriveKinematics(double lengthX, double lengthY) {
        this.TInv = new Matrix(new double[][]{
                {1, -1, -(lengthX + lengthY)},
                {1, 1, (lengthX + lengthY)},
                {1, 1, -(lengthX + lengthY)},
                {1, -1, (lengthX + lengthY)}
            }
        );
        this.lengthX = lengthX;
        this.lengthY = lengthY;
    }
    public double[] getPowers(double x, double y, double theta) {
        Matrix X = new Matrix(new double[][]{
                {x, y, theta}
        }).transpose();

        Matrix B = TInv.multiply(X);

        return new double[] {B.get(0, 0), B.get(1, 0), B.get(2, 0), B.get(3, 0)};
    }

    public void setLengthX(double lx) {lengthX = lx;}
    public void setLengthY(double ly) {lengthY = ly;}
}
