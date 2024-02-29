package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double output = 0;

    private DoubleSupplier getError;
    private double error;
    private double errorSum;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd, DoubleSupplier getError) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.getError = getError;
        this.error = getError.getAsDouble();
        timer.reset();
    }

    public void update() {
        double lastError = error;
        error = getError.getAsDouble();
        double derivative = (lastError - error) / timer.seconds();
        errorSum += error * timer.seconds();
        timer.reset();

        output = Kp * error + Ki * errorSum + Kd * derivative;
    }

    public double getOutput() {
        update();
        return output;
    }

    public void resetSum() {
        errorSum = 0;
    }
}
