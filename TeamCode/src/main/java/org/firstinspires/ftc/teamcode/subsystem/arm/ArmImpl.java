package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystem.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class ArmImpl implements Arm {
    private final DcMotor armMotor;
    private Claw claw = null;

    public static final int LOW_BACKBOARD = 100;
    public static final int HIGH_BACKBOARD = 400;

    private boolean targeting = true;

    public ArmImpl(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotor.class, "AM");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(int position) {
        double power = position < getTargetPosition() ? 1 : 0.1;

        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void updateClaw() {
        int position = getTargetPosition();
        if (claw != null && position < LOW_BACKBOARD)
            claw.setLift(MathUtils.normalize(position, LOW_BACKBOARD, HIGH_BACKBOARD, .55, .45));
    }

    public int getTargetPosition() {
        return armMotor.getTargetPosition();
    }

    public void changePosition(int deltaPos) {
        setPosition(armMotor.getTargetPosition() + deltaPos);
    }

    public void manualMove(double power){
        if(!targeting) {
            armMotor.setPower(power);
        }
    }

    public void addClaw(Claw claw) {
        this.claw = claw;
    }
}
