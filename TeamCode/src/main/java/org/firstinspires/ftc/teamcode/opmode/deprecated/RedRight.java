package org.firstinspires.ftc.teamcode.opmode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

@Deprecated
public class RedRight extends OpMode {
    SampleDrive drive;
    SampleClaw claw;
    SegmentedArm arm;
    TylerCV cv;
    String position;

    @Override
    public void init() {
        ThreadUtils.startThreads();
        drive = new SampleDrive(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
        cv = new TylerCV(hardwareMap, 2);
        claw.close();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Position", cv.getPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        position = cv.getPosition();

        if (position == null)
            position = "Middle";

        cv.stop();
        ThreadUtils.rest();
        arm.initialExtension();
        ThreadUtils.rest(2000);

        placePurple();
        placeYellow();
        park();
        ThreadUtils.rest(2000);
    }

    @Override
    public void loop() {

    }

    public void placePurple() {
        telemetry.addLine("Starting Purple");
        telemetry.update();
        arm.setUpperArm(Math.PI/36);
        arm.setForearm(Math.PI);
        arm.setWrist(3*Math.PI/2);
        ThreadUtils.rest(1000);
        drive.strafe(15);
        drive.cartesianMove(35, 60);
        drive.spinTo(Math.PI/2);

        switch(position) {
            case "Left":
                drive.cartesianMove(-25, 45);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(50, -30);
                break;
            case "Middle":
                drive.cartesianMove(0, 60);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(25, -45);
                break;
            case "Right":
                drive.cartesianMove(25, 45);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(0, -30);
        }
    }

    public void placeYellow() {
        telemetry.addLine("Starting Yellow");
        telemetry.update();

        arm.setCoordinate(-25, 15);
        arm.waitForArm();
        ThreadUtils.rest(500);
        drive.strafe(22);

        switch (position) {
            case "Left":
                drive.drive(20);
                break;
            case "Middle":
                break;
            case "Right":
                drive.drive(-20);
        }

        claw.openLeft();
        ThreadUtils.rest(500);
        drive.strafe(-10);
    }

    public void park() {
        telemetry.addLine("Park");
        telemetry.update();
        arm.setCoordinate(0, 50);
        switch (position) {
            case "Right":
                drive.cartesianMove(10, 70);
                break;
            case "Middle":
                drive.cartesianMove(10, 90);
                break;
            case "Left":
                drive.cartesianMove(10, 100);
        }

        arm.rest();
        arm.waitForArm();
        ThreadUtils.rest(4000);
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
