package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@TeleOp(name="Config Seg Pos")
public class SegArmPosConfig extends OpMode {
    private SegmentedArm arm;
    private Controller controller;

    @Override
    public void init() {
        arm = new SegmentedArm(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void start() {
        arm.setCoordinate(-20, 20);
    }

    @Override
    public void loop() {
        arm.changeCoordinate(-controller.getValue("LX")/2, -controller.getValue("LY")/2);

        telemetry.addData("Target X", arm.getTargetX());
        telemetry.addData("Target Y", arm.getTargetY());
    }
}
