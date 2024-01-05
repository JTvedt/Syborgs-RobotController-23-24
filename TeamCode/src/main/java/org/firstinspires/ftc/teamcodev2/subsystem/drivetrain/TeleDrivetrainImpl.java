package org.firstinspires.ftc.teamcodev2.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcodev2.controller.Controller;

public class TeleDrivetrainImpl extends DrivetrainImpl implements TeleDrivetrain {
    public TeleDrivetrainImpl(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void teleDrive(Controller controller) {
        teleDrive(controller, 0.7);
    }

    public void teleDrive(Controller controller, double power) {
        teleDrive(controller.getValue("LX"), controller.getValue("LY"), controller.getValue("RY"), power);
    }

    public void teleDrive(double drive, double strafe, double turn) {
        teleDrive(drive, strafe, turn, 0.7);
    }

    public void teleDrive(double drive, double strafe, double turn, double power) {

    }
}
