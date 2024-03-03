package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Auton Reg B-L")
public class RegAutonBL extends BaseOpMode {
    private String position;

    @Override
    public void init() {
        super.init();
        drive = new OdoDrive(hardwareMap);
        arm = new ActuatorArm(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        cv = new TylerCV(hardwareMap, 1);

        claw.close();
    }

    @Override
    public void start() {
        arm.setArm(Math.PI/48);
        ThreadUtils.rest(200);
        arm.setExtension(6);
        arm.setWrist(-Math.PI/2);
        ThreadUtils.rest(200);

        switch(position) {
            case "Left":
                drive.moveToPosition(-60, 60, -2*Math.PI/5);
                break;
            case "Middle":
                drive.moveToPosition(-20, 60, -2*Math.PI/9);
                break;
            case "Right":
                drive.moveToPosition(-8, 60, -2*Math.PI/5);
        }

        drive.waitForDrive(telemetry);
        claw.openRight();
        ThreadUtils.rest(200);
        arm.setExtension(0);
        arm.setArm(ActuatorArm.ARM_MAX);
        arm.setWrist(Math.PI/2);
        OdoDrive.MAX_SPEED = 0.4;

        switch (position) {
            case "Left":
                drive.moveToPosition(-90, 45, -Math.PI/2);
                break;
            case "Middle":
                drive.moveToPosition(-90, 55, -Math.PI/2);
                break;
            case "Right":
                drive.moveToPosition(-90, 65, -Math.PI/2);
        }

        drive.waitForDrive(telemetry);
        arm.waitForArm();
        claw.openLeft();
        ThreadUtils.rest(200);

        OdoDrive.MAX_SPEED = 0.7;
        drive.moveToPosition(-80, 100, 0);
        drive.waitForDrive();
        arm.rest();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
