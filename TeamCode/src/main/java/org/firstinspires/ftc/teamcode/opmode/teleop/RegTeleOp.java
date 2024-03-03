package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.subsystem.claw.SampleClaw;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain.OdoDrive;
import org.firstinspires.ftc.teamcode.subsystem.extra.DroneLauncher;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;

@TeleOp(name="TeleOp Regionals")
public class RegTeleOp extends BaseOpMode {
    private int extension = -1;
    private boolean riggingFlag = false;

    @Override
    public void init() {
        super.init();

        drive = new OdoDrive(hardwareMap);
        arm = new ActuatorArm(hardwareMap);
        claw = new SampleClaw(hardwareMap);
        launcher = new DroneLauncher(hardwareMap);
    }

    @Override
    public void start() {
        arm.basePosition();
        arm.setWrist(0);
    }

    @Override
    public void loop() {
        // Drivetrain
        drive.teleDrive(gamepad1, p1.holdingButton("RT") ? .3 : .9);

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
            new Thread(this::intermediateProcess).start();

        // Arm
        if (p1.holdingButton("LT")) {
            if (!p1.holdingButton("RT")) {
                if (p1.holdingButton("DU")) {
                    ActuatorArm.ARM_START -= Math.PI / 3 * timer.seconds();
                    arm.setArm(ActuatorArm.ARM_BASE);
                }
                if (p1.holdingButton("DD")) {
                    ActuatorArm.ARM_START += Math.PI / 3 * timer.seconds();
                    arm.setArm(ActuatorArm.ARM_BASE);
                }
            } else {
                if (p1.holdingButton("DU"))
                    ActuatorArm.EXTENSION_START -= 3 * timer.seconds();
                if (p1.holdingButton("DD"))
                    ActuatorArm.EXTENSION_START += 3 * timer.seconds();
            }
        }

        // Claw
        if (p1.pressingButton("LB"))
            claw.toggleLeft();
        if (p1.pressingButton("RB"))
            claw.toggleRight();

        // Extra
        if (!p1.holdingButton("LT")) {
            if (p1.pressingButton("DU")) {
                if (!riggingFlag) {
                    arm.startRigging();
                    riggingFlag = true;
                } else {
                    arm.finishRigging();
                }
            }

            if (p1.pressingButton("DD"))
                launcher.launch();
        }

        // Telemetry
        telemetry.addData("Drive Pos", drive.getCoord().toString());
        telemetry.addData("Heading", MathUtils.round(drive.odometry.getAngle(), 2));
        telemetry.addData("Real Heading", MathUtils.round(drive.getAngle(), 2));

        telemetry.addData("EL", drive.odometry.getEL().getCurrentPosition());
        telemetry.addData("ER", drive.odometry.getER().getCurrentPosition());
        telemetry.addData("EB", drive.odometry.getEB().getCurrentPosition());

        telemetry.addData("dx", drive.odometry.getDx());
        telemetry.addData("dy", drive.odometry.getDy());

        telemetry.addData("turn", drive.turn);
        telemetry.addData("timer", timer.seconds());
        telemetry.update();

        timer.reset();
    }

    public void startupProcess() {
        claw.open();
        arm.setWrist(ActuatorArm.WRIST_BASE);
    }

    public void intakeProcess() {
        claw.close();
        ThreadUtils.rest(200);
        arm.setWrist(0);
    }

    public void outtakeProcess() {
        riggingFlag = false;
        extension = -1;
        claw.open();
        ThreadUtils.rest(200);
        arm.basePosition();
        arm.setWrist(0);
    }

    public void intermediateProcess() {
        riggingFlag = false;

        if (extension == -1 || extension == 8) {
            arm.setArm(ActuatorArm.ARM_MAX);
            arm.setWrist(Math.PI / 2);
            arm.setExtension(0);
            extension = 0;
        } else if (extension == 0) {
            arm.setExtension(4);
            extension = 4;
        } else if (extension == 4) {
            arm.setExtension(8);
            extension = 8;
        }
    }
}