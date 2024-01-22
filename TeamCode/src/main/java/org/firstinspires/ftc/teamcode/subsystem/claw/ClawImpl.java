package org.firstinspires.ftc.teamcode.subsystem.claw;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ThreadUtils;


public class ClawImpl implements Claw {
    private ClawState leftState = ClawState.OPEN;
    private ClawState rightState = ClawState.OPEN;

    protected Servo leftServo;
    protected Servo rightServo;
    protected Servo wristServo;

    public final double LEFT_CLOSE_VALUE = 0.55;
    public final double RIGHT_CLOSE_VALUE = 0.38;
    public final double LEFT_OPEN_VALUE = 0.62;
    public final double RIGHT_OPEN_VALUE = 0.28;

    public final double BACK_POSITION = .50;
    public final double DOWN_POSITION = .52;

    public boolean open;

    public ClawImpl(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class,"LC");
        rightServo = hardwareMap.get(Servo.class,"RC");
        wristServo = hardwareMap.get(Servo.class, "CW");
    }

    public void setLeft(double position) {
        leftServo.setPosition(position);
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
        rightServo.setPosition(position);
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

    public void spinDown(){
        if(!open)
            wristServo.setPosition(DOWN_POSITION);
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

    public void spinBack(){
        if(!open)
            wristServo.setPosition(BACK_POSITION);
    }

    public void setLift(double position) {
        wristServo.setPosition(position);
    }

    public void toggleLift() {
        setLift(wristServo.getPosition() == DOWN_POSITION ? BACK_POSITION : DOWN_POSITION);
    }

    public void placePixel() {
        new Thread(() -> {
            open();
            ThreadUtils.rest(150);
            close();
        }).start();
    }
}