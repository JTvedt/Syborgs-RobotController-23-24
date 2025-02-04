package org.firstinspires.ftc.teamcode.opmode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.claw.LiftClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@Deprecated
public class ClawLiftConfig extends OpMode {
    private LiftClaw claw;
    private Controller controller;

    private double liftPosition = 0.5;

    @Override
    public void init() {
        claw = new LiftClaw(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        claw.setLift(liftPosition);

        telemetry.addData("Lift", MathUtils.round(liftPosition, 2));
        telemetry.update();

        double factor;

        if (controller.holdingButton("RT"))
            factor = .25;
        else if (controller.holdingButton("RB"))
            factor = .01;
        else
            factor = .05;

        if (controller.pressingButton("DU"))
            liftPosition -= factor;
        if (controller.pressingButton("DD"))
            liftPosition += factor;
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}