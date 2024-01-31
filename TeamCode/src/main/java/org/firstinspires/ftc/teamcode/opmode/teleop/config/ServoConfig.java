package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Servo")
public class ServoConfig extends OpMode {
    private Controller controller;
    private Servo forearm;
    private Servo wrist;
    private Servo left;
    private Servo right;

    @Override
    public void init() {
        controller = new Controller(gamepad1);

        forearm = hardwareMap.get(Servo.class, "AS");
        wrist = hardwareMap.get(Servo.class, "WS");
        left = hardwareMap.get(Servo.class, "LC");
        right = hardwareMap.get(Servo.class, "RC");
    }

    @Override
    public void loop() {
        if (controller.pressingButton("A"))
            forearm.setPosition(0.5);
        if (controller.pressingButton("B"))
            wrist.setPosition(0.5);
        if (controller.pressingButton("X"))
            left.setPosition(0.5);
        if (controller.pressingButton("Y"))
            right.setPosition(0.5);
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
