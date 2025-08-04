package org.firstinspires.ftc.teamcode.util.pathplanner;


class Node {
    int x, y;
    double g = Double.POSITIVE_INFINITY;
    Node parent = null;

    Node(int x, int y) {
        this.x = x;
        this.y = y;
    }

    double f(Node goal) {
        return g + dist(this, goal);
    }

    private static double dist(Node a, Node b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }
}
