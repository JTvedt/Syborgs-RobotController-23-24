package org.firstinspires.ftc.teamcode.subsystem.extra;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    private Servo launcher;

    public DroneLauncher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(Servo.class, "DL");
        launcher.setPosition(1);
    }

    public void launch() {
        launcher.setPosition(0);
    }
}
