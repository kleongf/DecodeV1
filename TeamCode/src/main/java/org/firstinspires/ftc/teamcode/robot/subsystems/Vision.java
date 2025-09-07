package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Vision extends Subsystem {
    private final Limelight3A limelight;
    private int id = 21;
    // 21: GPP
    // 22: PGP
    // 23: PPG
    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // apriltag pipeline is pipeline 3
        limelight.pipelineSwitch(3);
        limelight.start();
    }

    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            id = fiducialResults.get(0).getFiducialId();
        }
    }

    @Override
    public void start() {
        limelight.start();
    }

    public int getId() {return id;}
}
