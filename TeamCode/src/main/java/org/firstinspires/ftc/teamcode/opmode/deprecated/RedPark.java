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
public class RedPark extends OpMode {
    SampleDrive drive;
    SampleClaw claw;
    SegmentedArm arm;

    @Override
    public void init() {
        ThreadUtils.startThreads();
        drive = new SampleDrive(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        arm = new SegmentedArm(hardwareMap);
        claw.close();
    }

    @Override
    public void start() {
        arm.initialExtension();
        drive.strafe(15);
        ThreadUtils.rest(15000);
        drive.drive(-240);
        park();
    }

    @Override
    public void loop() {

    }

    public void park() {
        telemetry.addLine("Park");
        telemetry.update();

        arm.setUpperArm(0);
        arm.setForearm(5*Math.PI/6);
        arm.setWrist(5*Math.PI/4);
        arm.waitForArm();
        ThreadUtils.rest(4000);
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
