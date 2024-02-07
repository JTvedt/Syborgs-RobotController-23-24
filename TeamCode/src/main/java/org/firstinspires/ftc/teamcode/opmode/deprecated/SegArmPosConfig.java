package org.firstinspires.ftc.teamcode.opmode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@Deprecated
public class SegArmPosConfig extends OpMode {
    private SegmentedArm arm;
    private Controller controller;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        arm = new SegmentedArm(hardwareMap);
    }

    @Override
    public void start() {
        arm.setCoordinate(-20, 20);
    }

    @Override
    public void loop() {
        arm.changeCoordinate(controller.getValue("LX")/6, -controller.getValue("LY")/6);

        telemetry.addData("Target X", arm.getTargetX());
        telemetry.addData("Target Y", arm.getTargetY());
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
