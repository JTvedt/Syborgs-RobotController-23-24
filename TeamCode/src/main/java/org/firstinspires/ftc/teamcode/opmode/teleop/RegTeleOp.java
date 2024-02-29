package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@TeleOp(name="TeleOp Regionals")
public class RegTeleOp extends BaseOpMode {
    @Override
    public void init() {
        super.init();

        drive = new OdoDrive(hardwareMap);
        arm = new ActuatorArm(hardwareMap);
        claw = new SampleClaw(hardwareMap);
    }

    @Override
    public void start() {
        arm.basePosition();
        arm.setWrist(0);
    }

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
                new Thread(() -> intermediateProcess(8)).start();
            else if (p1.holdingButton("RT"))
                new Thread(() -> intermediateProcess(4)).start();
            else
                new Thread(() -> intermediateProcess(0)).start();

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

        // Telemetry
        telemetry.addData("Drive Pos", drive.getCoord().toString());
        telemetry.addData("Angle", MathUtils.round(drive.getAngle(), 2));
        telemetry.update();
    }

    public void startupProcess() {
        claw.open();
        arm.setWrist(ActuatorArm.WRIST_BASE);
    }

    public void intakeProcess() {
        claw.close();
        ThreadUtils.rest(400);
        arm.setWrist(0);
    }

    public void outtakeProcess() {
        claw.open();
        ThreadUtils.rest(400);
        arm.basePosition();
        arm.setWrist(0);
    }

    public void intermediateProcess(double length) {

    }
}