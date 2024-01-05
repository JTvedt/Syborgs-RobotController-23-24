package org.firstinspires.ftc.teamcodev2.subsystem.claw;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawImpl implements Claw {
    private Servo left;
    private Servo right;

    @Override
    public void open() {
        left.setPosition(0.2);
    }

    @Override
    public void close() {

    }
}
