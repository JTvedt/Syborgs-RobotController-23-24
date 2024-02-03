package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

@Autonomous(name="Auton Blue-L")
public class BlueLeft extends OpMode {
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
        claw.close();
    }

    @Override
    public void init_loop() {
//        telemetry.addData("Position", cv.getPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        position = "Left";
//        cv.stop();
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
        drive.cartesianMove(-30, 55);
        drive.spinTo(-Math.PI/2);

        switch(position) {
            case "Right":
                drive.cartesianMove(25, 15);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(-50, 0);
                break;
            case "Middle":
                drive.cartesianMove(0, 60);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(-25, -45);
                break;
            case "Left":
                drive.cartesianMove(-25, 25);
                claw.openRight();
                ThreadUtils.rest(500);
                drive.cartesianMove(0, -10);
        }
    }

    public void placeYellow() {
        telemetry.addLine("Starting Yellow");
        telemetry.update();

        arm.setCoordinate(-25, 10);
        arm.waitForArm();
        ThreadUtils.rest(500);
        drive.strafe(-22);

        switch (position) {
            case "Right":
                drive.drive(20);
                claw.openLeft();
                ThreadUtils.rest(500);
                drive.strafe(10);
                break;
            case "Middle":
                claw.openLeft();
                ThreadUtils.rest(500);
                drive.strafe(10);
                break;
            case "Left":
                drive.drive(-10);
                claw.openLeft();
                ThreadUtils.rest(500);
                drive.strafe(10);
        }
    }

    public void park() {
        telemetry.addLine("Park");
        telemetry.update();
        arm.setCoordinate(0, 50);
        switch (position) {
            case "Right":
                drive.cartesianMove(-10, 70);
                break;
            case "Middle":
                drive.cartesianMove(-10, 90);
                break;
            case "Left":
                drive.cartesianMove(-10, 100);
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
