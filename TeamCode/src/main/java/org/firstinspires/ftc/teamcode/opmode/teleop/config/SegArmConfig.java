package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;

@TeleOp(name="Config Seg Arm")
public class SegArmConfig extends OpMode {
    SegmentedArm arm;
    Controller controller;

    @Override
    public void init() {
        arm = new SegmentedArm(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        int deltaArm;
        if (controller.holdingButton("RT"))
            deltaArm = 50;
        else if (controller.holdingButton("LT"))
            deltaArm = 50;
        else
            deltaArm = 50;

        if (controller.pressingButton("Y"))
            arm.changePosition(deltaArm);
        if (controller.pressingButton("X"))
            arm.changePosition(-deltaArm);

        if (controller.pressingButton("B"))
            arm.manualMove(.7);
        else if (controller.pressingButton("A"))
            arm.manualMove(-.7);

        if (controller.releasingButton("B") || controller.releasingButton("A"))
            arm.manualMove(0);
    }
}
