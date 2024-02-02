package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class SegmentedArm extends ArmImpl {
    public static final double UPPER_ARM_LENGTH = 28.58; // Measured on robot
    public static final double FOREARM_LENGTH = 22.22; // Measured on robot
    public static double UPPER_ARM_START = 0;
    public static double FOREARM_START = Math.PI/8;
    public static final double WRIST_START = Math.PI/4;
    public static final double EXTENDED_FOREARM = Math.PI;
    public static final double EXTENDED_UPPER_ARM = -Math.PI/24;
    public static final double EXTENDED_WRIST = 3 * Math.PI/2 + EXTENDED_UPPER_ARM + Math.PI/12;
    public static final double TICKS_PER_RADIAN = 2072 / (2 * Math.PI); // From the GoBilda Website
    public static final double SERVO_PER_RADIAN = -1 / (3 * Math.PI/2);
    public static final double COORD_FACTOR = 0.3;
    public static final double PAUSE_THRESHOLD = Math.PI/4;

    private final DcMotor forearmMotor;
    private final Servo wristServo;
    private double upperArmAngle = UPPER_ARM_START;
    private double forearmAngle = FOREARM_START;
    private double wristAngle = WRIST_START;

    private double targetX;
    private double targetY;
    private boolean isBehind = false;
    private int targetPos = 0;
    private int foreTarget = 0;
    public double powerTemp;
    public boolean tempFlag = false;

    public SegmentedArm(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public SegmentedArm(HardwareMap hardwareMap, boolean reset) {
        super(hardwareMap, reset);

        forearmMotor = hardwareMap.get(DcMotor.class, "FA");
        forearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forearmMotor.setDirection(DcMotor.Direction.REVERSE);
        forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wristServo = hardwareMap.get(Servo.class, "WS");

        new Thread(this::moveArmToTarget).start();
        new Thread(this::moveForearmToTarget).start();
    }

    public void setUpperArm(double angle) {
        if (angle > Math.PI)
            angle = Math.PI;

        upperArmAngle = angle;
        setPosition((int)(TICKS_PER_RADIAN * (upperArmAngle - UPPER_ARM_START)));
    }

    @Override
    public void setPosition(int ticks) {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPos = ticks;
    }

    public void moveArmToTarget() {
        int currentPos = getCurrentPosition();
        double coefficient;
        double power;
        double deltaPos;

        while (ThreadUtils.isRunThread()) {
            currentPos = getCurrentPosition();
            coefficient = 1;

            if (targetPos < currentPos)
                coefficient *= -1;

            power = Math.max(Math.min(.002 * Math.pow(Math.abs(targetPos - currentPos), 1.008), 0.2), .01);

            if (Math.abs(targetPos - currentPos) < 8)
                coefficient *= .1;

            powerTemp = power * coefficient;
            armMotor.setPower(power * coefficient);
        }
    }

    public void moveForearmToTarget() {
        int currentPos = getCurrentPosition();
        double coefficient;
        double power;
        double deltaPos;

        while (ThreadUtils.isRunThread()) {
            currentPos = getForeCurrent();
            coefficient = 1;

            if (foreTarget < currentPos)
                coefficient *= -1;

            power = Math.max(Math.min(.002 * Math.pow(Math.abs(foreTarget - currentPos), 1.008), 0.2), .01);

            if (Math.abs(foreTarget - currentPos) < 8)
                coefficient *= .1;

            powerTemp = power * coefficient;
            forearmMotor.setPower(power * coefficient);
        }
    }

    @Override
    public int getTargetPosition() {
        return targetPos;
    }

    public void changeUpperArm(double angle) {
        setUpperArm(upperArmAngle + angle);
    }

    public void setForeMotor(int position) {
        foreTarget = position;
    }

    public void changeForeMotor(int position) {
        setForeMotor(position - getForeTarget());
    }

    public int getForeTarget() {
        return foreTarget;
    }

    public int getForeCurrent() {
        return forearmMotor.getCurrentPosition();
    }

    public void setWristServo(double position) {
        wristServo.setPosition(position);
    }

    public void changeWristServo(double position) {
        setWristServo(position - wristServo.getPosition());
    }

    public void setForearm(double angle) {
        forearmAngle = angle;
        setForeMotor((int)(1425.1 / (2*Math.PI) * (angle - FOREARM_START)));
    }

    public void changeForearm(double angle) {
        setForearm(forearmAngle + angle);
    }

    public void setWrist(double angle) {
        wristAngle = angle;
        setWristServo(1 + SERVO_PER_RADIAN * (wristAngle - WRIST_START));
    }

    public void changeWrist(double angle) {
        setWrist(wristAngle + angle);
    }

    public double getUpperArmAngle() {
        return upperArmAngle;
    }

    public double getForearmAngle() {
        return forearmAngle;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    /**
     * Set the arm to a position behind the robot
     * @param x cm behind the robot
     * @param y cm above arm base
     */
    public void setCoordinate(double x, double y) {
        if (x > 0)
            x = 0;

        if (x * x + y * y == 0)
            return;

        if (Math.hypot(x, y) > UPPER_ARM_LENGTH + FOREARM_LENGTH - COORD_FACTOR) {
            double coefficient = (UPPER_ARM_LENGTH + FOREARM_LENGTH - COORD_FACTOR) / Math.hypot(x, y);
            x *= coefficient;
            y *= coefficient;
        }

        if (Math.hypot(x, y) < UPPER_ARM_LENGTH - FOREARM_LENGTH + COORD_FACTOR) {
            double coefficient = (UPPER_ARM_LENGTH - FOREARM_LENGTH + COORD_FACTOR) / Math.hypot(x, y);
            x *= coefficient;
            y *= coefficient;
        }

        targetX = x;
        targetY = y;
        isBehind = true;

        double distance = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        if (angle < 0)
            angle = 2 * Math.PI + angle;

        // Solve for the angles of the triangle
        double upperAngle = angle - MathUtils.lawOfCosines(UPPER_ARM_LENGTH, distance, FOREARM_LENGTH);
        double foreAngle = 2*Math.PI - MathUtils.lawOfCosines(UPPER_ARM_LENGTH, FOREARM_LENGTH, distance);
        double wristAngle = Math.PI - (MathUtils.lawOfCosines(UPPER_ARM_LENGTH, FOREARM_LENGTH, distance) - upperAngle);

        setUpperArm(upperAngle);
        setForearm(foreAngle);
        setWrist(wristAngle);
    }

    public void setCoordinate(double x, double y, SampleClaw claw) {
        setCoordinate(x, y);
    }

    public void changeCoordinate(double x, double y) {
        if (!isBehind)
            return;

        setCoordinate(targetX + x, targetY + y);
    }

    public void cancelCoordinate() {
        isBehind = false;
    }

    public void changeCoordinate(double x, double y, SampleClaw claw) {
        if (!isBehind)
            return;

        setCoordinate(targetX + x, targetY + y, claw);
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    /**
     * Round start, extend the arm
     */
    public void initialExtension() {
        new Thread(() -> {
            setUpperArm(2*Math.PI/3);
            setForearm(EXTENDED_FOREARM);
            waitForArm();
            ThreadUtils.rest(400);
            setWrist(EXTENDED_WRIST);
            setUpperArm(EXTENDED_UPPER_ARM);
        }).start();
    }

    public void extend() {
        cancelCoordinate();
        setUpperArm(EXTENDED_UPPER_ARM);
        setForearm(EXTENDED_FOREARM);
        setWrist(EXTENDED_WRIST);
    }

    public void contract() {
        cancelCoordinate();
        setUpperArm(0);
        setForearm(7 * Math.PI/4);
    }

    public void rest() {
        cancelCoordinate();
        setUpperArm(Math.PI/3);
        setForearm(5*Math.PI/6);
        setWrist(5*Math.PI/4);
        waitForArm();
        ThreadUtils.rest(500);
        setUpperArm(0);
    }
}
