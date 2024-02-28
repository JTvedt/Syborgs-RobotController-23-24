package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Belt Test")
public class BeltDriveTest extends LinearOpMode {
    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;
    @Override
    public void runOpMode()
    {

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


        double x,y,turn,theta,power,sin,cos,max;

        double frontLeft, frontRight, backLeft, backRight;

        waitForStart();

        while (opModeIsActive())
        {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            theta = Math.atan2(y,x);
            power = Math.hypot(x,y);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin),Math.abs(cos));

            frontLeft = power * cos/max + turn;
            frontRight = power * sin/max - turn;
            backLeft = power * sin/max + turn;
            backRight = power * cos/max - turn;

            if((power + Math.abs(turn)) > 1)
            {
                frontLeft /= power + turn;
                frontRight /= power + turn;
                backLeft /= power + turn;
                backRight /= power + turn;
            }
            FL.setPower(frontLeft);
            FR.setPower(frontRight);
            BL.setPower(backLeft);
            BR.setPower(backRight);

        }
    }
}
