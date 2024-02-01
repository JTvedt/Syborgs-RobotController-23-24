package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.extra.DroneLauncher;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Drone")
public class DroneConfig extends OpMode {
    DroneLauncher launcher;
    Controller controller;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        launcher = new DroneLauncher(hardwareMap);
    }

    @Override
    public void loop() {
        if (controller.pressingButton("A"))
            launcher.launch();
    }

    public void stop() {
        ThreadUtils.stopThreads();
    }
}
