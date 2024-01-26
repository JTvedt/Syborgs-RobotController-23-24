package org.firstinspires.ftc.teamcode.subsystem.claw;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ThreadUtils;


public class LiftClaw extends SampleClaw {
    private ClawState leftState = ClawState.OPEN;
    private ClawState rightState = ClawState.OPEN;

    protected Servo liftServo;
    public final double BACK_POSITION = .50;
    public final double DOWN_POSITION = .52;

    public boolean open;

    public LiftClaw(HardwareMap hardwareMap){
        super(hardwareMap);
        liftServo = hardwareMap.get(Servo.class, "WS");
    }

    public void spinDown(){
        liftServo.setPosition(DOWN_POSITION);
    }

    public void spinBack(){
        liftServo.setPosition(BACK_POSITION);
    }

    public void setLift(double position) {
        liftServo.setPosition(position);
    }

    public void toggleLift() {
        setLift(liftServo.getPosition() == DOWN_POSITION ? BACK_POSITION : DOWN_POSITION);
    }
}