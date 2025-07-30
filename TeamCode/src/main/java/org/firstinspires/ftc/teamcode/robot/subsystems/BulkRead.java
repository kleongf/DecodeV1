package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class BulkRead {
    private List<LynxModule> allHubs;
    public BulkRead(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void start() {

    }
}
