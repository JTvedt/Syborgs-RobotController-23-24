package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
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

    boolean toggle = false;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        drive = new SampleDrive(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);
        SegmentedArm.FOREARM_START = 5*Math.PI/6;
    }

    @Override
    public void start() {
        claw.open();
        toggle = false;
        arm.extend();
        intakeProcess();
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1, p1.holdingButton("RT") ? 0.2 : 0.85);

        // Processes
        if (p1.pressingButton("X"))
            if (p1.holdingButton("RT") && p1.holdingButton("LT"))
                intermediateProcess(-20, 45);
            else if (p1.holdingButton("RT"))
                intermediateProcess(-30, 30);
            else
                intermediateProcess(-30, 15);
        if (p1.pressingButton("B") || p2.pressingButton("B"))
            outtakeProcess();
        if (p1.pressingButton("Y"))
            intakeProcess();
        if (p1.pressingButton("A") || p2.pressingButton("A"))
            claw.toggle();

        // Fine tuning
        if (p1.pressingButton("LB") || p2.pressingButton("LB"))
            claw.toggleLeft();
        if (p1.pressingButton("RB") || p2.pressingButton("RB"))
            claw.toggleRight();

        double val = Math.PI/72;
        if (p2.holdingButton("LT"))
            val = Math.PI/8;

        if (p2.holdingButton("RT")) {
            if (p2.pressingButton("DU")) {
                SegmentedArm.FOREARM_START -= val;
                returnPos();
            }

            if (p2.pressingButton("DD")) {
                SegmentedArm.FOREARM_START += val;
                arm.extend();
            }
        } else {
            if (p2.pressingButton("DU")) {
                SegmentedArm.UPPER_ARM_START -= val;
                arm.extend();
            }

            if (p2.pressingButton("DD")) {
                SegmentedArm.UPPER_ARM_START += val;
                arm.extend();
            }
        }

        if (drive.getAngle() < 0)
            arm.changeCoordinate(2 * p2.getValue("LX")/3, -2 * p2.getValue("LY")/3);
        else
            arm.changeCoordinate(-2 * p2.getValue("LX")/3, -2 * p2.getValue("LY")/3);

        if (p1.pressingButton("LS"))
            drive.setAnchorAngle();

        // Extra
        if (p1.pressingButton("DU"))
            ;// Rigging here
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
            arm.extend();
            arm.setUpperArm(0);
            toggle = false;
            intakeProcess();
        }).start();
    }

    private void intakeProcess() {
        new Thread(() -> {
            if (toggle) {
                arm.extend();
                claw.open();
                toggle = false;
            } else {
                claw.close();
                ThreadUtils.rest(500);
                arm.setUpperArm(Math.PI/24);
                toggle = true;
            }
        }).start();
    }

    private void returnPos() {
        if (toggle)
            arm.setUpperArm(Math.PI/24);
        else
            arm.extend();}
}
