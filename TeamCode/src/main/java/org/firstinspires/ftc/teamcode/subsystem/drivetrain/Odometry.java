package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface Odometry {
    double getX();
    double getY();
    double getAngle();
}
