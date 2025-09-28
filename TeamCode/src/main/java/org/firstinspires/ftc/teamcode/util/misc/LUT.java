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
        if (data.isEmpty()) {
            throw new IllegalStateException("Lookup table is empty.");
        }

        if (data.containsKey(query)) {
            return data.get(query);
        }

        if (query <= data.firstKey()) {
            return data.get(data.firstKey());
        }

        if (query >= data.lastKey()) {
            return data.get(data.lastKey());
        }

        Double lowerX = data.floorKey(query);   // <= query
        Double higherX = data.ceilingKey(query); // >= query

        if (lowerX == null || higherX == null) {
            throw new IllegalStateException("Unexpected: no bounding keys found.");
        }

        double lowerY = data.get(lowerX);
        double higherY = data.get(higherX);

        return lowerY + (query - lowerX) * ((higherY - lowerY) / (higherX - lowerX));
    }


//    public double getValue(double query) {
//        if (data.containsKey(query)) {
//            return data.get(query);
//        }
//
//        // if the thing is out of range, then choose the closest one
//        if (query > data.lastKey()) {
//            return data.get(data.lastKey());
//        }
//
//        if (query < data.firstKey()) {
//            return data.get(data.firstKey());
//        }
//
//        //return data.get(data.firstKey());
//
//        double lowerX = data.lowerKey(query);
//        double higherX = data.higherKey(query);
//        double lowerY = data.get(lowerX);
//        double higherY = data.get(higherX);
//
//        return lowerY + (query-lowerX) * ((higherY-lowerY) / (higherX-lowerX));
//    }
}

