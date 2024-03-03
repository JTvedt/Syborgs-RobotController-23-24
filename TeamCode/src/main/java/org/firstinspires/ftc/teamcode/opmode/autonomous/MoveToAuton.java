package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@Autonomous(name="Auton Move")
public class MoveToAuton extends BaseOpMode {
    @Override
    public void init() {
        super.init();
        drive = new OdoDrive(hardwareMap);
    }

    @Override
    public void start() {
        drive.moveToPosition(-60, 60, Math.PI/2);
    }
}
