package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;

@TeleOp(name="TeleOp Regionals")
public class RegTeleOp extends BaseOpMode {
    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1);

        if (p1.pressingButton("LS"))
            drive.resetAngle();

        // Processes
        if (p1.pressingButton("Y"))
            new Thread(this::startupProcess).start();
        if (p1.pressingButton("A"))
            new Thread(this::intakeProcess).start();
        if (p1.pressingButton("B"))
            new Thread(this::outtakeProcess).start();
        if (p1.pressingButton("X"))
            if (p1.holdingButton("RT") && p1.holdingButton("LT"))
                new Thread(() -> intermediateProcess(-20, 45)).start();
            else if (p1.holdingButton("RT"))
                new Thread(() -> intermediateProcess(-30, 30)).start();
            else
                new Thread(() -> intermediateProcess(-30, 15)).start();

        // Arm


        // Claw
        if (p1.pressingButton("LB"))
            claw.toggleLeft();
        if (p1.pressingButton("RB"))
            claw.toggleRight();
        
        // Extra
        if (p1.pressingButton("DU"))
            ; // Rigging
        if (p1.pressingButton("DD"))
            ; // Drone Launcher
    }

    public void startupProcess() {
        claw.open();
    }

    public void intakeProcess() {
        claw.close();
    }

    public void outtakeProcess() {

    }

    public void intermediateProcess(double x, double y) {

    }
}