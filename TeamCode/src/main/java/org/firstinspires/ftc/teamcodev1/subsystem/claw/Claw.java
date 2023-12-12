package org.firstinspires.ftc.teamcodev1.subsystem.claw;

public interface Claw {
    void setLeft(double position);
    void openLeft();
    void closeLeft();

    void setRight(double position);
    void openRight();
    void closeRight();

    void open();
    void close();

    void setLift(double position);
}
