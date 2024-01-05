package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor intakeLift;
    private final CRServo intakeSpin;

    public final int INTAKE_CLOSE = -100;
    public final int INTAKE_OPEN = -12;

    private State state = State.OPEN;

    public enum State {
        OPEN,
        CLOSE;
    }

    public Intake(HardwareMap hardwareMap) {
        intakeLift = hardwareMap.get(DcMotor.class,"IM");
        intakeSpin = hardwareMap.get(CRServo.class, "IS");

        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void close(){
        intakeLift.setTargetPosition(INTAKE_CLOSE);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(0.8);
        state = State.CLOSE;
    }

    public void open() {
        intakeLift.setTargetPosition(INTAKE_OPEN);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(0.8);
        state = State.OPEN;
    }

    public void toggle() {
        if (state.equals(State.OPEN))
            close();
        else
            open();
    }

    public void spin(double power){
        intakeSpin.setPower(power);
    }
}
