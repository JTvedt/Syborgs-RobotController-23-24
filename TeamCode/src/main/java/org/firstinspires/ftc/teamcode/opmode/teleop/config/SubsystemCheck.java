package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Subsystem Check")
public class SubsystemCheck extends OpMode {
    Controller controller;
    ActuatorArm arm;
    SampleClaw claw;
    int a = 0;

    @Override
    public void init() {
        controller = new Controller(gamepad1);

        arm = new ActuatorArm(hardwareMap);
        claw = new SampleClaw(hardwareMap);
    }

    @Override
    public void loop() {
        if (controller.pressingButton("A")) {
            a++;
            switch (a) {
                case 1:
                    claw.openLeft();
                    break;
                case 2:
                    claw.openRight();
                    break;
                case 3:
                    arm.setWrist(0);
                    break;
                case 4:
                    arm.setArm(0);
                    break;
                case 5:
                    arm.setExtension(2);
                    break;
                case 6:
                    arm.setExtension(0);
                    break;
            }
        }
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
