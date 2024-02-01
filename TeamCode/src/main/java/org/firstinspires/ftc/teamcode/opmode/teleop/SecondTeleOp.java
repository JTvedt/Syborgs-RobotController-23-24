package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.LiftClaw;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.subsystem.extra.DroneLauncher;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="TeleOp Second")
public class SecondTeleOp extends OpMode {
    SampleDrive drive;
    SampleClaw claw;
    SegmentedArm arm;
    DroneLauncher launcher;

    Controller p1;
    Controller p2;

    int color = 0;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        drive = new SampleDrive(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);
    }

    @Override
    public void start() {
        arm.extend();
        claw.open();
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1, p1.holdingButton("RT") ? 0.2 : 0.7);

        // Processes
        if (p1.pressingButton("X"))
            if (p1.holdingButton("RT") && p1.holdingButton("LT"))
                intermediateProcess(-20, 45);
            else if (p1.holdingButton("RT"))
                intermediateProcess(-30, 35);
            else
                intermediateProcess(-30, 25);
        if (p1.pressingButton("B"))
            outtakeProcess();
        if (p1.pressingButton("A"))
            claw.toggle();

        // Fine tuning
        if (p1.pressingButton("LB"))
            claw.toggleLeft();
        if (p1.pressingButton("RB"))
            claw.toggleRight();

        if (p2.pressingButton("DU")) {
            SegmentedArm.UPPER_ARM_START -= Math.PI / 72;
            arm.extend();
        }

        if (p2.pressingButton("DD")) {
            SegmentedArm.UPPER_ARM_START += Math.PI/72;
            arm.extend();
        }

        if (drive.getAngle() < 0)
            arm.changeCoordinate(p2.getValue("LX")/3, -p2.getValue("LY")/3);
        else
            arm.changeCoordinate(-p2.getValue("LX")/3, -p2.getValue("LY")/3);

        // Extra
        if (p1.pressingButton("DU"))
            ;// Linear Actuator here
        if (p1.pressingButton("DD"))
            launcher.launch();

        telemetry.addData("RT", String.valueOf(p1.holdingButton("RT")));
        telemetry.addData("Arm Current", arm.getCurrentPosition());
        telemetry.addData("Arm Target", arm.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }

    private void intermediateProcess(double x, double y) {
        arm.setCoordinate(x, y);
    }

    private void outtakeProcess() {
        new Thread(() -> {
            claw.open();
            ThreadUtils.rest(500);
            arm.extend();
            arm.setUpperArm(Math.PI/3);
            arm.waitForArm();
            ThreadUtils.rest(500);
            arm.setUpperArm(SegmentedArm.EXTENDED_UPPER_ARM);
            arm.extend();
        }).start();
    }
}
