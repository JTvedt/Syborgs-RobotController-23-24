package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Claw Sample")
public class SampleClawConfig extends OpMode {
    SampleClaw claw;
    Controller controller;

    private double leftPos = 0.5;
    private double rightPos = 0.5;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        claw = new SampleClaw(hardwareMap);
    }

    @Override
    public void loop() {
        claw.setLeft(leftPos);
        claw.setRight(rightPos);

        double deltaServo;
        if (controller.holdingButton("RT"))
            deltaServo = 0.25;
        else if (controller.holdingButton("LT"))
            deltaServo = 0.01;
        else
            deltaServo = 0.05;

        if (controller.pressingButton("Y"))
            leftPos += deltaServo;
        if (controller.pressingButton("X"))
            leftPos -= deltaServo;
        if (controller.pressingButton("B"))
            rightPos += deltaServo;
        if (controller.pressingButton("A"))
            rightPos -= deltaServo;

        telemetry.addData("Left", leftPos);
        telemetry.addData("Right", rightPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}