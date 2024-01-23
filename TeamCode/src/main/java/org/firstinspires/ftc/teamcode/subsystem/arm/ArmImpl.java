package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystem.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class ArmImpl implements Arm {
    protected final DcMotor armMotor;
    protected Claw claw = null;

    public static final int LOW_BACKBOARD = 100;
    public static final int HIGH_BACKBOARD = 400;

    public ArmImpl(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotor.class, "RA");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setPosition(0);
    }

    public void setPosition(int position) {
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
    }

    public void updateClaw() {
        int position = getTargetPosition();
        if (claw != null && position < LOW_BACKBOARD)
            claw.setLift(MathUtils.normalize(position, LOW_BACKBOARD, HIGH_BACKBOARD, .55, .45));
    }

    public int getTargetPosition() {
        return armMotor.getTargetPosition();
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public void changePosition(int deltaPos) {
        setPosition(armMotor.getTargetPosition() + deltaPos);
    }

    public void manualMove(double power){

        armMotor.setPower(power);
    }

    public void addClaw(Claw claw) {
        this.claw = claw;
    }
}
