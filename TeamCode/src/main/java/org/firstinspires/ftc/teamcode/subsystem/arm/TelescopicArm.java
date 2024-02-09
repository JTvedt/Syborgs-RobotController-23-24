package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public class TelescopicArm extends ArmImpl {
    public static final double ARM_START = 0;
    public static final double TICKS_PER_REVOLUTION = 1425.1 * 2;

    private Servo wrist;
    private DcMotor extender;
    private DcMotor leftArm;

    private double tx = 0;
    private double ty = 0;
    private boolean toCoord = false;

    private int armTarget;

    public TelescopicArm(HardwareMap hardwareMap) {
        super(hardwareMap);

        leftArm = hardwareMap.get(DcMotor.class, "LA");
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        wrist = hardwareMap.get(Servo.class, "WS");
        extender = hardwareMap.get(DcMotor.class, "EM");

        new Thread(this::updateArm).start();
    }

    public void updateArm() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController leftPID = new PIDController(0, 0, 0, getTargetPosition() - leftArm.getCurrentPosition());
        PIDController rightPID = new PIDController(0, 0, 0, getTargetPosition() - armMotor.getCurrentPosition());

        while (ThreadUtils.isRunThread()) {
            leftPID.update(getTargetPosition() - leftArm.getCurrentPosition());
            leftArm.setPower(Range.clip(leftPID.getOutput(), -.3, .3));

            rightPID.update(getTargetPosition() - armMotor.getCurrentPosition());
            armMotor.setPower(Range.clip(leftPID.getOutput(), -.3, .3));
        }
    }

    @Override
    public int getTargetPosition() {
        return armTarget;
    }

    @Override
    public int getCurrentPosition() {
        return (armMotor.getCurrentPosition() + leftArm.getCurrentPosition())/2;
    }

    public void setExtension(double length) {

    }

    public void setArm(double angle) {
        setPosition((int)(TICKS_PER_REVOLUTION * (angle - ARM_START)/(2*Math.PI)));
    }

    @Override
    public void setPosition(int position) {
        armTarget = position;
    }

    public void setWrist(double angle) {

    }

    public void setCoordinate(double x, double y) {
        tx = x;
        ty = y;
        toCoord = true;

        double theta = Math.atan2(y, x);
        double length = Math.hypot(x, y);

        setArm(Math.PI - theta);
        setExtension(length);
        setWrist(5*Math.PI/6 - theta);
    }

    public void changeCoordinate(double x, double y) {
        if (!toCoord)
            return;

        setCoordinate(tx + x, ty + y);
    }
}
