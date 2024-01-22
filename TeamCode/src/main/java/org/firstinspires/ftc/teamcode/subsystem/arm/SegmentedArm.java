package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.claw.WristClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

public class SegmentedArm extends ArmImpl {
    // TODO measure these on the robot
    public static double UPPER_ARM_LENGTH = 0;
    public static double FOREARM_LENGTH = 0;
    public static double UPPER_ARM_START = -Math.PI/2;
    public static double FOREARM_START = Math.PI/6;
    public static double EXTENDED_FOREARM = .67;
    public static double EXTENDED_UPPER_ARM = -50;
    public static double TICKS_PER_RADIAN = 0;
    public static double SERVO_PER_RADIAN = 1 / (3 * Math.PI/2);

    protected Servo armServo;
    private double upperArmAngle = UPPER_ARM_START;
    private double forearmAngle = FOREARM_START;

    public SegmentedArm(HardwareMap hardwareMap) {
        super(hardwareMap);

        armServo = hardwareMap.get(Servo.class, "AS");
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
    }

    public void setForearm(double angle) {
        forearmAngle = angle;
        setServo(SERVO_PER_RADIAN * (forearmAngle - FOREARM_START));
    }

    public void changeForearm(double angle) {
        setForearm(forearmAngle + angle);
    }

    /**
     * Set the arm to a position behind the robot
     * @param x cm behind the robot
     * @param y cm above the ground
     */
    public void setCoordinate(int x, int y) {

    }

    public void setCoordinate(int x, int y, WristClaw claw) {
        setCoordinate(x, y);
    }

    public void extendArm(WristClaw claw) {
        new Thread(() -> {
            setForearm(EXTENDED_FOREARM);
            ThreadUtils.rest();
            setUpperArm(EXTENDED_UPPER_ARM);
        }).start();
    }
}
