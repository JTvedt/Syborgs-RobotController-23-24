package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="TeleOp B2")
public class B2TeleOp extends OpMode {
    SampleDrive drive;
    ClawImpl claw;
    SegmentedArm arm;

    Controller p1;
    Controller p2;

    @Override
    public void init() {
        p1 = new Controller(gamepad1);
        p2 = new Controller(gamepad2);

        drive = new SampleDrive(hardwareMap);
//        claw = new ClawImpl(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
    }

    @Override
    public void start() {
        arm.initialExtension();
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1);

        // Processes
        if (p1.pressingButton("X"))
            if (p1.holdingButton("RT"))
                intermediateProcess(-30, 40);
            else
                intermediateProcess(-25, -20);
        if (p1.pressingButton("B"))
            outtakeProcess();
        if (p1.pressingButton("A"))
            intakeProcess();

        // Fine tuning
        arm.changeCoordinate(p2.getValue("LX")/2, -p2.getValue("LY")/2);

        // Extra
        if (p1.pressingButton("DU"))
            ;// Linear Actuator here
        if (p1.pressingButton("DD"))
            ;// Drone Launcher here

        telemetry.addData("RT", String.valueOf(p1.holdingButton("RT")));
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }

    private void intakeProcess() {
        new Thread(() -> {
            arm.contract();
        }).start();
    }

    private void intermediateProcess(double x, double y) {
        arm.setCoordinate(x, y);
    }

    private void outtakeProcess() {
        arm.extend();
    }
}
