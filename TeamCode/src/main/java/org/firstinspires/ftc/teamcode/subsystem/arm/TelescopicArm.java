package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TelescopicArm extends ArmImpl {
    private Servo wrist;
    private DcMotor extender;

    private double tx = 0;
    private double ty = 0;
    private boolean toCoord = false;

    public TelescopicArm(HardwareMap hardwareMap) {
        super(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "WS");
        extender = hardwareMap.get(DcMotor.class, "EM");
    }

    public void setExtension(double length) {

    }

    public void setArm(double angle) {

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
