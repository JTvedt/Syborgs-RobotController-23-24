package org.firstinspires.ftc.teamcode.subsystem.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class WristClaw extends ClawImpl {
    // TODO measure these!!!
    public static double WRIST_START = 0;
    public static double SERVO_PER_ANGLE = 1 / Math.PI;

    public WristClaw(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void setWrist(double angle) {
        wristServo.setPosition(SERVO_PER_ANGLE * (angle - WRIST_START));
    }
}
