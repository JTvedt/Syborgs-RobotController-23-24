package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.claw.WristClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class SegmentedArm extends ArmImpl {
    public static double UPPER_ARM_LENGTH = 28.58; // Measured on robot
    public static double FOREARM_LENGTH = 22.22; // Measured on robot
    public static double UPPER_ARM_START = 0;
    public static double FOREARM_START = Math.PI/4; // From gobilda website
    public static double EXTENDED_FOREARM = Math.PI;
    public static double EXTENDED_UPPER_ARM = -Math.PI/180;
    public static double TICKS_PER_RADIAN = 2072 / (2 * Math.PI); // From the GoBilda Website
    public static double SERVO_PER_RADIAN = 1 / (3 * Math.PI/2);
    public static double COORD_FACTOR = 0.3;
    public static double PAUSE_THRESHOLD = Math.PI/4;

    protected Servo armServo;
    private double upperArmAngle = UPPER_ARM_START;
    private double forearmAngle = FOREARM_START;

    private double targetX;
    private double targetY;
    private boolean isBehind = false;
    private int targetPos = 0;
    public double powerTemp;
    public boolean tempFlag = false;

    public SegmentedArm(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public SegmentedArm(HardwareMap hardwareMap, boolean reset) {
        super(hardwareMap, reset);

        armServo = hardwareMap.get(Servo.class, "AS");

        new Thread(this::goToTarget).start();
        setForearm(FOREARM_START);
    }

    public void updatePower() {
        double currentAngle;
        double targetAngle;
        double power;
        double coefficient;
        int currentPos;
        int targetPos;

        while (ThreadUtils.isRunThread()) {
            currentPos = getCurrentPosition();
            targetPos = getTargetPosition();
            currentAngle = currentPos / TICKS_PER_RADIAN;
            targetAngle = targetPos / TICKS_PER_RADIAN;

            if (currentAngle < targetAngle)
                power = MathUtils.normalize(currentAngle, 0, Math.PI, .3, .01);
            else
                power = MathUtils.normalize(currentAngle, Math.PI, 0, .3, .01);

            coefficient = MathUtils.normalize(Math.abs(targetPos - currentPos), Math.PI/24, Math.PI/2, 0.01, 1);


            if (Math.abs(targetAngle - Math.PI/2) < 3*Math.PI/7)
                if ((currentPos > targetPos && currentPos - targetPos < 5)
                        || (currentPos < targetPos && targetPos  - currentPos < 5)
                        || currentPos == targetPos)
                    coefficient = 0.01;

            powerTemp = power * coefficient;
            armMotor.setPower(power * coefficient);
        }
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

    public void goToTarget() {
        int currentPos = getCurrentPosition();
        double coefficient;
        double power;
        double deltaPos;

        while (ThreadUtils.isRunThread()) {
            deltaPos = currentPos - getCurrentPosition();
            currentPos = getCurrentPosition();
            coefficient = 1;

            if (targetPos < currentPos)
                coefficient *= -1;

            power = Math.max(Math.min(.002 * Math.pow(Math.abs(targetPos - currentPos), 1.008), 0.3), .01);

            if (Math.abs(targetPos - currentPos) < 15)
                coefficient *= .1;

            powerTemp = power * coefficient;
            armMotor.setPower(power * coefficient);
        }
    }

    @Override
    public int getTargetPosition() {
        return targetPos;
    }

    public void changeUpperArm(double angle) {
        setUpperArm(upperArmAngle + angle);
    }

    public void setServo(double position) {
        armServo.setPosition(position);
    }

    public void changeServo(double position) {
        setServo(position - armServo.getPosition());
    }

    public void setForearm(double angle) {

        forearmAngle = angle;
        setServo(1 - SERVO_PER_RADIAN * (forearmAngle - FOREARM_START));
    }

    public void changeForearm(double angle) {
        setForearm(forearmAngle + angle);
    }

    public double getServoPosition() {
        return armServo.getPosition();
    }

    public double getUpperArmAngle() {
        return upperArmAngle;
    }

    public double getForearmAngle() {
        return forearmAngle;
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

        setUpperArm(upperAngle);
        setForearm(foreAngle);
    }

    public void setCoordinate(double x, double y, WristClaw claw) {
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

    public void changeCoordinate(double x, double y, WristClaw claw) {
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
            setUpperArm(Math.PI/6);
            setForearm(EXTENDED_FOREARM);
            ThreadUtils.rest(700);
            setUpperArm(EXTENDED_UPPER_ARM);
        }).start();
    }

    public void extend() {
        cancelCoordinate();
        setUpperArm(EXTENDED_UPPER_ARM);
        setForearm(EXTENDED_FOREARM);
    }

    public void contract() {
        cancelCoordinate();
        setUpperArm(0);
        setForearm(7 * Math.PI/4);
    }

    public void waitForArm() {
        while (Math.abs(getCurrentPosition() - getTargetPosition()) > 5) {

        }
    }
}
