package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Drivetrain Simple")
public class DrivetrainConfigSimple extends OpMode {
    Controller controller;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void init() {
        controller = new Controller(gamepad1);

        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
    }

    @Override
    public void loop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        if (gamepad1.x)
            fl.setPower(1);
        if (gamepad1.y)
            fr.setPower(1);
        if (gamepad1.a)
            bl.setPower(1);
        if (gamepad1.b)
            br.setPower(1);

        if (controller.getValue("LY") != 0) {
            fl.setPower(controller.getValue("LY"));
            fr.setPower(controller.getValue("LY"));
            bl.setPower(controller.getValue("LY"));
            br.setPower(controller.getValue("LY"));
        }
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
