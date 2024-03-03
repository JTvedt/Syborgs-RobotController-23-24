package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.subsystem.extra.DroneLauncher;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="TeleOp TeleDrive")
public class TeleDriveTeleOp extends OpMode {
    SampleDrive drive;

    Controller p1;
    Controller p2;

    boolean toggle = false;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        drive = new SampleDrive(hardwareMap);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1, p1.holdingButton("RT") ? 0.2 : 0.85);
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
