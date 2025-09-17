package org.firstinspires.ftc.teamcode.util.misc;

import java.util.TreeMap;

public class LUT {
    private TreeMap<Double, Double> data;
    public LUT() {
        this.data = new TreeMap<>();
    }
    public void addData(double x, double fx) {
        data.put(x, fx);
    }

    public double getValue(double query) {
        if (data.containsKey(query)) {
            return data.get(query);
        }

        // if the thing is out of range, then choose the closest one
        if (query > data.lastKey()) {
            return data.get(data.lastKey());
        }

        if (query < data.firstKey()) {
            return data.get(data.firstKey());
        }

        double lowerX = data.lowerKey(query);
        double higherX = data.higherKey(query);
        double lowerY = data.get(lowerX);
        double higherY = data.get(higherX);

        return lowerY + (query-lowerX) * ((higherY-lowerY) / (higherX-lowerX));
    }
}

