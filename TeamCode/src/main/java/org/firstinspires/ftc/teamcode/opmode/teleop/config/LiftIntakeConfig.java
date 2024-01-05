package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Lift Intake")
public class LiftIntakeConfig extends OpMode {
    private int position = 0;
    private DcMotor intakeLift;
    private Controller controller;

    @Override
    public void init() {
        intakeLift = hardwareMap.get(DcMotor.class, "IM");
        controller = new Controller(gamepad1);

        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setTargetPosition(position);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        intakeLift.setTargetPosition(position);
        intakeLift.setPower(0.5);

        int factor;

        if (controller.holdingButton("RT"))
            factor = 25;
        else if (controller.holdingButton("RB"))
            factor = 1;
        else
            factor = 5;

        if (controller.pressingButton("X"))
            position += factor;
        if (controller.pressingButton("Y"))
            position -= factor;

        telemetry.addData("Position", position);
        telemetry.addData("Target Position", intakeLift.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
