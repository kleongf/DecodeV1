package org.firstinspires.ftc.teamcode.util.hardware;

import com.pedropathing.localization.Pose;

import java.util.ArrayList;
import java.util.Arrays;

public class SimplePathChain {
    private ArrayList<SimplePath> paths;
    public SimplePathChain(SimplePath...paths) {
        this.paths = new ArrayList<>(Arrays.asList(paths));
    }
    public SimplePath getPath(int i) {
        return paths.get(i);
    }

    public int getSize() {
        return paths.size();
    }
}
