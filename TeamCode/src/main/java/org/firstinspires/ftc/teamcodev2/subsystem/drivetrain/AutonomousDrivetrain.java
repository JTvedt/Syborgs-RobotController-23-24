package org.firstinspires.ftc.teamcodev2.subsystem.drivetrain;

public interface AutonomousDrivetrain extends Drivetrain {
    void polarMove(double distance, double angle);
    void cartesianMove(double x, double y);
    void strafe(double distance);
    void drive(double distance);

    void spinTo(double angle);
    void spin(double angle);
}