package org.firstinspires.ftc.teamcode.util.pathplanner;

import java.util.*;

public class LazyTheta {
    private static final int WIDTH = 144;
    private static final int HEIGHT = 144;
    private int ROBOT_RADIUS = 9;
    private final boolean[][] obstacles = new boolean[WIDTH][HEIGHT];
    private final Node[][] nodes = new Node[WIDTH][HEIGHT];

//    static class Node {
//        int x, y;
//        double g = Double.POSITIVE_INFINITY;
//        Node parent = null;
//
//        Node(int x, int y) {
//            this.x = x;
//            this.y = y;
//        }
//
//        double f(Node goal) {
//            return g + heuristic(this, goal);
//        }
//    }

    public LazyTheta(int radius) {
        for (int i = 0; i < WIDTH; i++)
            for (int j = 0; j < HEIGHT; j++)
                nodes[i][j] = new Node(i, j);

        ROBOT_RADIUS = radius;
    }

    private void addObstacle(int x, int y) {
        if (inBounds(x, y))
            obstacles[x][y] = true;
    }

    public void addObstacle(int topLeftX, int topLeftY, int width, int height) {
        for (int i = topLeftX-ROBOT_RADIUS; i <= topLeftX+ROBOT_RADIUS+width; i++) {
            for (int j = topLeftY-ROBOT_RADIUS-height; j <= topLeftY+ROBOT_RADIUS; j++) {
                addObstacle(i, j);
            }
        }
    }

    public List<Node> findPath(int sx, int sy, int gx, int gy) {
        if (obstacles[sx][sy] || obstacles[gx][gy]) return Collections.emptyList();

        Node start = nodes[sx][sy];
        Node goal = nodes[gx][gy];
        start.g = 0;
        start.parent = start;

        PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingDouble(n -> n.f(goal)));
        boolean[][] closed = new boolean[WIDTH][HEIGHT];
        open.add(start);

        while (!open.isEmpty()) {
            Node current = open.poll();
            if (current == goal) return reconstructPath(goal);

            closed[current.x][current.y] = true;

            for (Node neighbor : getNeighbors(current)) {
                if (closed[neighbor.x][neighbor.y]) continue;

                if (!open.contains(neighbor)) {
                    neighbor.g = Double.POSITIVE_INFINITY;
                    neighbor.parent = null;
                }

                if (lineOfSight(current.parent, neighbor)) {
                    double newG = current.parent.g + dist(current.parent, neighbor);
                    if (newG < neighbor.g) {
                        neighbor.g = newG;
                        neighbor.parent = current.parent;
                        open.remove(neighbor);
                        open.add(neighbor);
                    }
                } else {
                    double newG = current.g + dist(current, neighbor);
                    if (newG < neighbor.g) {
                        neighbor.g = newG;
                        neighbor.parent = current;
                        open.remove(neighbor);
                        open.add(neighbor);
                    }
                }
            }
        }
        return Collections.emptyList(); // no path found
    }

    private List<Node> reconstructPath(Node goal) {
        List<Node> path = new ArrayList<>();
        Node curr = goal;
        while (curr != curr.parent) {
            path.add(curr);
            curr = curr.parent;
        }
        path.add(curr);
        Collections.reverse(path);
        return path;
    }

    private boolean lineOfSight(Node a, Node b) {
        int x0 = a.x, y0 = a.y;
        int x1 = b.x, y1 = b.y;

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x1 > x0 ? 1 : -1;
        int sy = y1 > y0 ? 1 : -1;

        int err = dx - dy;

        while (x0 != x1 || y0 != y1) {
            if (obstacles[x0][y0]) return false;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
        return !obstacles[x1][y1];
    }

    private List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        int[] dx = {-1, -1, -1, 0, 1, 1, 1, 0};
        int[] dy = {-1, 0, 1, 1, 1, 0, -1, -1};

        for (int i = 0; i < dx.length; i++) {
            int nx = node.x + dx[i];
            int ny = node.y + dy[i];
            if (inBounds(nx, ny) && !obstacles[nx][ny]) {
                neighbors.add(nodes[nx][ny]);
            }
        }
        return neighbors;
    }

    private static boolean inBounds(int x, int y) {
        return x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT;
    }

    private static double dist(Node a, Node b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    private static double heuristic(Node a, Node b) {
        return dist(a, b);
    }
}