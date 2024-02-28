package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ExposedDrive extends SampleDrive {
    public ExposedDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public DcMotor getFL() { return motorFL; }
    public DcMotor getFR() { return motorFR; }
    public DcMotor getBL() { return motorBL; }
    public DcMotor getBR() { return motorBR; }

    public List<DcMotor> getList() { return motorList; }

    @Override
    public void setRunMode(DcMotor.RunMode runMode) {
        super.setRunMode(runMode);
    }

    @Override
    public void setPower(double power) {
        super.setPower(power);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        super.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
