package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

@Autonomous(name="Auton Blue-L")
public class BlueLeft extends LinearOpMode {
    SampleDrive drive;
    SampleClaw claw;
    SegmentedArm arm;
    TylerCV cv;
    String position;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleDrive(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
        cv = new TylerCV(hardwareMap, 1);

        while (!isStarted()) {
            Vector target = new Vector(0, 60);
            target.rotate(0);
            telemetry.addData("Angle", drive.getAngle());
            telemetry.addData("Position", cv.getPosition());
            telemetry.update();
        }

        position = "Middle";
        cv.stop();

        arm.initialExtension();
        ThreadUtils.rest(2000);

        placePurple();
        placeYellow();
        park();
        ThreadUtils.rest(2000);
    }

    public void placePurple() {
        telemetry.addLine("Starting Purple");
        telemetry.update();
        arm.setUpperArm(0);
        drive.cartesianMove(-30, 60);
        drive.spinTo(-Math.PI/2);

        switch(position) {
            case "Right":
                drive.cartesianMove(22.5, 25);
                claw.openRight();
                drive.cartesianMove(-50, 0);
                break;
            case "Middle":
                drive.cartesianMove(0, 55);
                claw.openRight();
                drive.cartesianMove(-25, -30);
                break;
            case "Left":
                drive.cartesianMove(-25, 25);
                claw.openRight();
        }
    }

    public void placeYellow() {
        telemetry.addLine("Starting Yellow");
        telemetry.update();

        arm.setCoordinate(-25, 20);
        arm.waitForArm();
        ThreadUtils.rest(500);
        drive.strafe(-30);

        switch (position) {
            case "Right":
                claw.openLeft();
                ThreadUtils.rest(500);
                drive.drive(-20);
                break;
            case "Middle":
                drive.drive(-20);
                claw.openLeft();
                ThreadUtils.rest(500);
                break;
            case "Left":
                drive.drive(-40);
                claw.openLeft();
                ThreadUtils.rest(500);
                drive.drive(20);
        }
    }

    public void park() {
        telemetry.addLine("Park");
        telemetry.update();
        arm.setCoordinate(0, 50);
        drive.cartesianMove(-10, 70);
        arm.rest();
        arm.waitForArm();
    }
}
