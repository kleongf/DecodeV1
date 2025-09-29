package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad {
    private Gamepad gamepad;

    // Previous state for edge detection
    private double rightTriggerPrev, leftTriggerPrev;
    private double triggerThreshold = 0.05;

    public SmartGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    // Call this once per loop to update internal state
    public void update() {
        rightTriggerPrev = gamepad.right_trigger;
        leftTriggerPrev = gamepad.left_trigger;
    }

    // Edge detection for buttons
    public boolean aPressed() {
        return gamepad.aWasPressed();
    }

    public boolean bPressed() {
        return gamepad.bWasPressed();
    }

    public boolean xPressed() {
        return gamepad.xWasPressed();
    }

    public boolean yPressed() {
        return gamepad.yWasPressed();
    }

    public boolean leftBumperPressed() {return gamepad.leftBumperWasPressed();}

    public boolean rightBumperPressed() {
        return gamepad.rightBumperWasPressed();
    }

    public boolean dpadUpPressed() {
        return gamepad.dpadUpWasPressed();
    }

    public boolean dpadDownPressed() {
        return gamepad.dpadDownWasPressed();
    }

    public boolean dpadLeftPressed() {
        return gamepad.dpadLeftWasPressed();
    }

    public boolean dpadRightPressed() {return gamepad.dpadRightWasPressed();}

    public boolean leftTriggerPressed() {
        return gamepad.left_trigger > triggerThreshold && !(leftTriggerPrev > triggerThreshold);
    }

    public boolean rightTriggerPressed() {
        return gamepad.right_trigger > triggerThreshold && !(rightTriggerPrev > triggerThreshold);
    }

    // Analog stick accessors
    public float getLeftStickX() { return gamepad.left_stick_x; }
    public float getLeftStickY() { return gamepad.left_stick_y; }
    public float getRightStickX() { return gamepad.right_stick_x; }
    public float getRightStickY() { return gamepad.right_stick_y; }

    public float getLeftTrigger() { return gamepad.left_trigger; }
    public float getRightTrigger() { return gamepad.right_trigger; }

    // Current state passthrough (optional)
    public boolean a() { return gamepad.a; }
    public boolean b() { return gamepad.b; }
    public boolean x() { return gamepad.x; }
    public boolean y() { return gamepad.y; }
}
