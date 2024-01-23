package org.firstinspires.ftc.teamcode.subsystem.arm;

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
    public static double EXTENDED_UPPER_ARM = -Math.PI/8;
    public static double TICKS_PER_RADIAN = 1425.1 / (2 * Math.PI); // From the GoBilda Website
    public static double SERVO_PER_RADIAN = 1 / (3 * Math.PI/2);

    protected Servo armServo;
    private double upperArmAngle = UPPER_ARM_START;
    private double forearmAngle = FOREARM_START;

    private double targetX;
    private double targetY;
    private boolean isBehind = false;
    public double servoTemp;

    public SegmentedArm(HardwareMap hardwareMap) {
        super(hardwareMap);
        new Thread(this::updatePower).start();

        armServo = hardwareMap.get(Servo.class, "AS");
        setForearm(FOREARM_START);
    }

    public void updatePower() {
        double currentAngle;
        double targetAngle;
        double power;
        double coefficient;

        while (ThreadUtils.isRunThread()) {
            currentAngle = armMotor.getCurrentPosition() / TICKS_PER_RADIAN;
            targetAngle = armMotor.getTargetPosition() / TICKS_PER_RADIAN;

            if (currentAngle < targetAngle)
                power = MathUtils.normalize(currentAngle, 0, Math.PI, .3, .01);
            else
                power = MathUtils.normalize(currentAngle, Math.PI, 0, .3, .01);

            coefficient = MathUtils.normalize(Math.abs(targetAngle - currentAngle), 0, Math.PI/2, 0.3, 1);

            armMotor.setPower(power * coefficient);
        }
    }

    public void setUpperArm(double angle) {
        upperArmAngle = angle;
        setPosition((int)(TICKS_PER_RADIAN * (upperArmAngle - UPPER_ARM_START)));
    }

    public void changeUpperArm(double angle) {
        setUpperArm(upperArmAngle + angle);
    }

    public void setServo(double position) {
        armServo.setPosition(position);
        servoTemp = position;
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
        if (x < 0)
            x = 0;

        if (x * x + y * y == 0)
            return;

        if (Math.hypot(x, y) > UPPER_ARM_LENGTH + FOREARM_LENGTH - 1) {
            double coefficient = (UPPER_ARM_LENGTH + FOREARM_LENGTH - 1) / Math.hypot(x, y);
            x *= coefficient;
            y *= coefficient;
        }

        if (Math.hypot(x, y) < UPPER_ARM_LENGTH - FOREARM_LENGTH + 1) {
            double coefficient = (UPPER_ARM_LENGTH - FOREARM_LENGTH + 1) / Math.hypot(x, y);
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
    public void extendArm() {
        new Thread(() -> {
            setUpperArm(Math.PI/6);
            setForearm(EXTENDED_FOREARM);
            ThreadUtils.rest();
            setUpperArm(EXTENDED_UPPER_ARM);
        }).start();
    }
}
