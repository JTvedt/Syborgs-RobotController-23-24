package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Seg Wrist")
public class SegArmWristConfig extends OpMode {
    private SegmentedArm arm;
    private Controller controller;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        arm = new SegmentedArm(hardwareMap);
        arm.initialExtension();
    }

    @Override
    public void loop() {
        double deltaWrist;
        if (controller.holdingButton("RT"))
            deltaWrist = Math.PI/2;
        else if (controller.holdingButton("LT"))
            deltaWrist = Math.PI/18;
        else
            deltaWrist = Math.PI/6;

        if (controller.pressingButton("B"))
            arm.changeWrist(deltaWrist);
        else if (controller.pressingButton("A"))
            arm.changeWrist(-deltaWrist);

        telemetry.addData("Wrist Angle", arm.getWristAngle());
        telemetry.addData("Wrist Position", arm.wristServo.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
