package org.firstinspires.ftc.teamcodev2.subsystem.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class DrivetrainImpl implements Drivetrain {
    protected DcMotor motorFL;
    protected DcMotor motorFR;
    protected DcMotor motorBL;
    protected DcMotor motorBR;
    protected List<DcMotor> motorList;

    protected BNO055IMU imu;

    public DrivetrainImpl(HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "FL");
        motorBR = hardwareMap.get(DcMotor.class, "FL");

        motorList = Arrays.asList(motorFL, motorFR, motorBL, motorBR);
    }

    public double getAngle() {
        return 0;
    }

    public boolean isMoving() {
        for (DcMotor motor : motorList)
            if (motor.isBusy())
                return true;
        return false;
    }

    protected void setRunMode(DcMotor.RunMode mode) {
        motorList.forEach(motor -> motor.setMode(mode));
    }

    protected void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorList.forEach(motor -> motor.setZeroPowerBehavior(zeroPowerBehavior));
    }

    protected void setPower(double power) {
        motorList.forEach(motor -> motor.setPower(power));
    }
}
