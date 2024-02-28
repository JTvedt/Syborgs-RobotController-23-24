package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

public interface DrivetrainMecanum extends DrivetrainSimple {
    void strafe(double cm);
    void cartesianMove(double cmX, double cmY);
    double getAngle();
}
