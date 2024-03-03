package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.subsystem.extra.DroneLauncher;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

public class BaseOpMode extends OpMode {
    protected Controller p1;
    protected Controller p2;

    protected OdoDrive drive;
    protected ActuatorArm arm;
    protected SampleClaw claw;
    protected DroneLauncher launcher;
    protected ElapsedTime timer;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
