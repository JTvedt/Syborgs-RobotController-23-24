package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.ArmImpl;
import org.firstinspires.ftc.teamcode.subsystem.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="TeleOp Second")
public class SecondTeleOp extends OpMode {
    SampleDrive drive;
    ClawImpl claw;
    ArmImpl arm;

    Controller p1;
    Controller p2;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        drive = new SampleDrive(hardwareMap);
        claw = new ClawImpl(hardwareMap);
        arm = new ArmImpl(hardwareMap);
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1);

        // Processes
        if (p1.pressingButton("A") && p1.holdingButton("RT"))
            intakeProcess(ArmImpl.HIGH_BACKBOARD);
        if (p1.pressingButton("A") && !p1.holdingButton("RT"))
            intakeProcess(ArmImpl.LOW_BACKBOARD);
        if (p1.pressingButton("B"))
            outtakeProcess();

        // Subsystems
        if (p1.holdingButton("Y"))
            arm.changePosition(1);
        if (p1.holdingButton("X"))
            arm.changePosition(-1);

        // Extra
        if (p1.pressingButton("DU"))
            ;// Linear Actuator here
        if (p1.pressingButton("DD"))
            ;// Drone Launcher here
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }

    private void intakeProcess(int height) {

    }

    private void outtakeProcess() {

    }
}
