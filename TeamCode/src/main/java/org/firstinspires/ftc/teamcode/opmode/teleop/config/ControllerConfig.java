package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Controller")
public class ControllerConfig extends OpMode {
    private Controller controller;

    @Override
    public void init() {
        controller = new Controller(gamepad1, telemetry);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
