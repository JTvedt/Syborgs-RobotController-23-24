package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeleOpDrive{
    //Creates 4 variables to represent the 4 DcMotors
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;

    public TeleOpDrive(HardwareMap hardwareMap){
        FR = hardwareMap.get(DcMotor.class, "FR");
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL = hardwareMap.get(DcMotor.class, "FL");
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR = hardwareMap.get(DcMotor.class, "BR");
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL = hardwareMap.get(DcMotor.class, "BL");
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motor reverse for right side
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
    }

    public void teleDrive(double x, double y, double spin, Telemetry telemetry){
        telemetry.addData("Drive Input X", x);
        telemetry.addData("Drive Input Y", y);

        FL.setPower(x + y - spin);
        FR.setPower(x - y + spin);
        BL.setPower(x - y - spin);
        BR.setPower(x + y + spin);

        telemetry.update();
    }
}
