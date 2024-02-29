package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public class ActuatorArm extends ArmImpl {
    public static final double ARM_START = -Math.PI/5;
    public static final double WRIST_START = -3*Math.PI/4;

    public static final double ARM_BASE = -Math.PI/16;
    public static final double ARM_MAX = 11*Math.PI/18;
    public static final double WRIST_BASE = -Math.PI/2 - ARM_BASE;

    public static final double TICKS_PER_REVOLUTION = 5500;
    public static final double SERVO_PER_RAD = 1 / (3*Math.PI/2);
    public static final double TICKS_PER_CM = 500;

    public static final double BASE_LENGTH = 14.3 * 2.54;

    private final Servo wrist;
    public final DcMotor actuator;

    private double tx = 0;
    private double ty = 0;
    private boolean toCoord = false;

    private int armTarget;
    private int extensionTarget;

    private boolean resetArmPID;
    private boolean resetActuatorPID;

    public ActuatorArm(HardwareMap hardwareMap) {
        super(hardwareMap);

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        wrist = hardwareMap.get(Servo.class, "WS");

        actuator = hardwareMap.get(DcMotor.class, "Lift");
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        new Thread(this::updateArm).start();
    }

    public void updateArm() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController armPID = new PIDController(.0025, .0008, 0, () -> getTargetPosition() - getCurrentPosition());
        PIDController actuatorPID = new PIDController(.004, 0, 0, () -> extensionTarget - actuator.getCurrentPosition());

        while (ThreadUtils.isRunThread()) {
            armMotor.setPower(Range.clip(armPID.getOutput(), -.5, .5));
            actuator.setPower(Range.clip(actuatorPID.getOutput(), -.8, .8));

            if (resetArmPID) {
                armPID.resetSum();
                resetArmPID = false;
            }

            if (resetActuatorPID) {
                actuatorPID.resetSum();
                resetActuatorPID = false;
            }
        }
    }

    @Override
    public int getTargetPosition() {
        return armTarget;
    }

    public int getExtensionTarget() {
        return extensionTarget;
    }

    public void setExtension(double length) {
        extensionTarget = (int)(length * TICKS_PER_CM);
        resetActuatorPID = true;
//        actuator.setTargetPosition(extensionTarget);
//        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        actuator.setPower(.8);
    }

    public void changeExtension(double distance) {
        setExtension(extensionTarget + distance);
    }

    public void setArm(double angle) {
        if (angle > ARM_MAX)
            angle = ARM_MAX;

        setPosition((int)(TICKS_PER_REVOLUTION * (angle - ARM_START)/(2*Math.PI)));
    }

    @Override
    public void setPosition(int position) {
        armTarget = position;
        resetArmPID = true;
//        armMotor.setTargetPosition(position);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.2);
    }

    public void setWrist(double angle) {
        wrist.setPosition(SERVO_PER_RAD * (angle - WRIST_START));
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

    public void basePosition() {
        setExtension(0);
        setArm(ARM_BASE);
        setWrist(WRIST_BASE);
    }

    public void rest() {
        setArm(ARM_START);
        setWrist(Math.PI/4);
    }
}
