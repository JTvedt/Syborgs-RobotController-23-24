package org.firstinspires.ftc.teamcode.opmode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.claw.LiftClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@Deprecated
public class ClawPositionConfig extends OpMode {
    private Controller controller;
    private LiftClaw claw;

    private double leftPosition = .5;
    private double rightPosition = .5;

    @Override
    public void init() {
        claw = new LiftClaw(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        claw.setLeft(leftPosition);
        claw.setRight(rightPosition);

        telemetry.addData("Left", MathUtils.round(leftPosition, 2));
        telemetry.addData("Right", MathUtils.round(rightPosition, 2));
        telemetry.update();

        double factor;

        if (controller.holdingButton("RT"))
            factor = .25;
        else if (controller.holdingButton("RB"))
            factor = .01;
        else
            factor = .05;

        if (controller.pressingButton("X"))
            leftPosition += factor;
        if (controller.pressingButton("Y"))
            rightPosition -= factor;
        if (controller.pressingButton("A"))
            leftPosition -= factor;
        if (controller.pressingButton("B"))
            rightPosition += factor;
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}