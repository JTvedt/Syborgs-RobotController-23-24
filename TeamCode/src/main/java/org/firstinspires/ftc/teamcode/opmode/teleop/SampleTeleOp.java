package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.ArmImpl;
import org.firstinspires.ftc.teamcode.subsystem.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

@TeleOp(name="TeleOp Sample")
public class SampleTeleOp extends OpMode {
    SampleDrive drive;
    ClawImpl claw;
    ArmImpl arm;
    Intake intake;
    Controller controller;

    private boolean intakeFlag = false;

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
        claw = new ClawImpl(hardwareMap);
        arm = new ArmImpl(hardwareMap);
        intake = new Intake(hardwareMap);

        controller = new Controller(gamepad1);
    }

    @Override
    public void start() {
        claw.open();
        claw.setLift(claw.DOWN_POSITION);
        intake.open();
        intakeFlag = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Stick X", gamepad1.left_stick_x);
        telemetry.addData("Stick Y", gamepad1.left_stick_y);
        telemetry.addData("R-Stick X", gamepad1.right_stick_x);
        drive.teleDrive(gamepad1);

        Vector targetVector = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        targetVector.rotate(-drive.getAngle());

        telemetry.addData("Angle", drive.getAngle());
        telemetry.addData("Target X", targetVector.getX());
        telemetry.addData("Target Y", targetVector.getY());
        telemetry.update();

        if (controller.pressingButton("LT"))
            claw.toggleLeft();
        if (controller.pressingButton("RT"))
            claw.toggleRight();
        if (controller.pressingButton("A"))
            claw.toggle();
        if (controller.pressingButton("RB"))
            claw.toggleLift();

        if (controller.pressingButton("DU")) {
            arm.setPosition(-400);
            claw.setLift(claw.BACK_POSITION);
        }

        if (controller.pressingButton("DD")) {
            arm.setPosition(0);
            claw.setLift(claw.DOWN_POSITION);
        }

        if (controller.pressingButton("DL"))
            arm.setPosition(-100);
        if (controller.pressingButton("DR"))
            arm.setPosition(10);

        if (controller.pressingButton("LB"))
            intake.toggle();
        if (controller.holdingButton("X"))
            intake.spin(1);
        else if (!intakeFlag)
            intake.spin(0);

        if (controller.pressingButton("B"))
            intakeProcess();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }

    private void intakeProcess() {
        intakeFlag = true;
        new Thread(() -> {
            intake.spin(1);
            ThreadUtils.rest(400);
            claw.close();
            intake.spin(0);

            ThreadUtils.rest(3000);

            arm.setPosition(-300);
            claw.setLift(claw.BACK_POSITION);

            intakeFlag = false;
        }).start();
    }
}
