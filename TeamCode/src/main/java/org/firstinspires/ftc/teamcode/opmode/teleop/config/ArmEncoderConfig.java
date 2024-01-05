package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.ArmImpl;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Arm Encoder")
public class ArmEncoderConfig extends OpMode {
    private ArmImpl arm;
    private Controller controller;
    private int position = 0;

    @Override
    public void init() {
        arm = new ArmImpl(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        int factor;

        if (controller.holdingButton("RT"))
            factor = 10;
        else if (controller.holdingButton("RB"))
            factor = 250;
        else
            factor = 50;

        if (controller.pressingButton("X"))
            position -= factor;
        if (controller.pressingButton("Y"))
            position += factor;

        arm.setPosition(position);

        telemetry.addData("Position", position);
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
