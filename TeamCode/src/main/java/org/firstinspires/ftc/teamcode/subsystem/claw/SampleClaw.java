package org.firstinspires.ftc.teamcode.subsystem.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public class SampleClaw implements Claw {
    private ClawState leftState = ClawState.OPEN;
    private ClawState rightState = ClawState.OPEN;

    protected Servo leftClaw;
    protected Servo rightClaw;

    public static final double LEFT_CLOSE_VALUE = 0.35;
    public static final double RIGHT_CLOSE_VALUE = 0.7;
    public static final double LEFT_OPEN_VALUE = 0.43;
    public static final double RIGHT_OPEN_VALUE = 0.62;

    private boolean open;

    public SampleClaw(HardwareMap hardwareMap){
        leftClaw = hardwareMap.get(Servo.class,"LC");
        rightClaw = hardwareMap.get(Servo.class,"RC");

        close();
    }

    public void setLeft(double position) {
        leftClaw.setPosition(position);
    }

    public void openLeft() {
        setLeft(LEFT_OPEN_VALUE);
        leftState = ClawState.OPEN;
    }

    public void closeLeft() {
        setLeft(LEFT_CLOSE_VALUE);
        leftState = ClawState.CLOSE;
    }

    public void setRight(double position) {
        rightClaw.setPosition(position);
    }

    public void openRight() {
        setRight(RIGHT_OPEN_VALUE);
        rightState = ClawState.OPEN;
    }

    public void closeRight() {
        setRight(RIGHT_CLOSE_VALUE);
        rightState = ClawState.CLOSE;
    }

    @Override
    public void open(){
        openLeft();
        openRight();
        open = true;
    }

    @Override
    public void close(){
        closeLeft();
        closeRight();
        open = false;
    }

    public void toggleLeft() {
        if (leftState.equals(ClawState.OPEN))
            closeLeft();
        else
            openLeft();
    }

    public void toggleRight() {
        if (rightState.equals(ClawState.OPEN))
            closeRight();
        else
            openRight();
    }

    public void toggle() {
        toggleLeft();
        toggleRight();
    }

    public void placePixel() {
        new Thread(() -> {
            open();
            ThreadUtils.rest(150);
            close();
        }).start();
    }
}
