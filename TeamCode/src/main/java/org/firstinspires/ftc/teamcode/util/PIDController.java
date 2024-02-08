package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;

    private double output = 0;

    private double error;
    private double errorSum;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD, double error) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.error = error;
        timer.reset();
    }

    public void update(double error) {
        double lastError = this.error;
        this.error = error;
        double derivative = (lastError - error) / timer.seconds();
        errorSum += error * timer.seconds();
        timer.reset();

        output = kP * error + kI * errorSum + kD * derivative;
    }

    public double getOutput() {
        return output;
    }
}
