package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Auton Reg B-R")
public class RegAutonBR extends BaseOpMode {
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
    public void init_loop() {
        telemetry.addData("Position", cv.getPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        position = cv.getPosition();
        cv.stop();

        arm.setArm(Math.PI/48);
        ThreadUtils.rest(200);
        arm.setExtension(6);
        arm.setWrist(-Math.PI/2);
        ThreadUtils.rest(200);

        switch(position) {
            case "Right":
                drive.moveToPosition(40, 40, 1*Math.PI/5);
                break;
            case "Middle":
                drive.moveToPosition(20, 60, 2*Math.PI/9);
                break;
            case "Left":
                drive.moveToPosition(8, 60, 2*Math.PI/5);
        }

        drive.waitForDrive(telemetry);
        claw.openRight();
        ThreadUtils.rest(200);
        arm.basePosition();
        arm.setArm(Math.PI/48);
        drive.moveToPosition(0, 10, -Math.PI/2);
        drive.moveToCoord(-240, 0);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
