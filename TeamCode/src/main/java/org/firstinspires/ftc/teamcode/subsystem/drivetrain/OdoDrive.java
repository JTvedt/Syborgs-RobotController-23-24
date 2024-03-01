package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

public class OdoDrive extends SampleDrive {
    private Vector targetPos = new Vector(0, 0);
    private double targetAngle = 0;
    private boolean isTarget = false;
    public final Odometry odometry;

    @Override
    public void teleDrive(double lStickX, double lStickY, double rStickX, double power) {
        if (lStickX != 0 || lStickY != 0 || rStickX != 0)
            isTarget = false;
        super.teleDrive(lStickX, lStickY, rStickX, power);
    }

    public OdoDrive(HardwareMap hardwareMap) {
        super(hardwareMap, true);
        odometry = new OdoImpl(hardwareMap);

        new Thread(this::updatePosition).start();
    }

    public void updatePosition() {
        PIDController positionPID = new PIDController(0, 0, 0, () -> distanceTo(targetPos));
        PIDController anglePID = new PIDController(0, 0, 0, () -> distanceToAngle(targetAngle));

        while (ThreadUtils.isRunThread()) {
            if (isTarget) {
                double turn = anglePID.getOutput();
                double drivePower = Math.max(Math.abs(positionPID.getOutput()), .7);

                Vector targetVector = targetPos.subtract(getCoord());
                targetVector.multiply(1 / targetVector.hypot());
                targetVector.multiply(drivePower);

                motorFL.setPower(targetVector.getX() + targetVector.getY() + turn);
                motorFR.setPower(-targetVector.getX() + targetVector.getY() - turn);
                motorBL.setPower(-targetVector.getX() + targetVector.getY() + turn);
                motorBR.setPower(targetVector.getX() + targetVector.getY() - turn);
            }
        }
    }

    public void waitForDrive() {
        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < 3 && ThreadUtils.isRunThread())
            if (distanceTo(targetPos) > .3 || Math.abs(distanceToAngle(targetAngle)) > Math.PI/180)
                timer.reset();
    }

    public Vector getCoord() {
        return new Vector(odometry.getX(), odometry.getY());
    }

    @Override
    public double getAngle() {
        return super.getAngle();
    }

    public void moveToPosition(Vector targetPos, double angle) {
        this.targetPos = targetPos;
        targetAngle = angle;
        isTarget = true;
    }

    public void moveToPosition(double x, double y, double angle) {
        moveToPosition(new Vector(x, y), angle);
    }

    public void moveToCoord(Vector targetPos) {
        moveToPosition(targetPos, targetAngle);
    }

    public void moveToCoord(double x, double y) {
        moveToPosition(x, y, targetAngle);
    }

    @Override
    public void spinTo(double rad) {
        moveToPosition(targetPos, rad);
    }

    public double distanceTo(Vector target) {
        Vector curPos = getCoord();
        return Math.hypot(target.getX() - curPos.getX(), target.getY() - curPos.getY());
    }
}
