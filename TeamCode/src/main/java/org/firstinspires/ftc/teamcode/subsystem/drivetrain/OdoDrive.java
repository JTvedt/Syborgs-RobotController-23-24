package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;
import org.firstinspires.ftc.teamcode.util.math.Vector;

public class OdoDrive extends SampleDrive {
    public static double MAX_SPEED = 0.7;

    private Vector targetPos = new Vector(0, 0);
    private double targetAngle = 0;
    public double turnOutput;
    public Vector targetVector;
    private boolean isTarget = false;
    public final OdoImpl odometry;

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
        PIDController positionPID = new PIDController(.05, 0, 0, () -> distanceTo(targetPos));
        PIDController anglePID = new PIDController(.5, 0, 0, () -> distanceToAngle(targetAngle));

        while (ThreadUtils.isRunThread()) {
            if (isTarget) {
                turnOutput = anglePID.getOutput();

                if (Math.abs(turnOutput) < .05 && Math.abs(distanceToAngle(targetAngle)) < Math.PI/72)
                    turnOutput *= .05/turnOutput;

                double drivePower = Range.clip(Math.abs(positionPID.getOutput()), 0, MAX_SPEED);

//                if (Math.abs(distanceToAngle(targetAngle)) > Math.PI/90 && Math.abs(turnOutput) < .1)
//                    turnOutput *= (.1 / turnOutput);
//                if (Math.abs(distanceTo(targetPos)) > 1.5 && Math.abs(drivePower) < .1)
//                    drivePower *= (.1 / drivePower);

                targetVector = targetPos.copy();
                targetVector.subtract(this.getCoord());
                targetVector.rotate(-getAngle());
                targetVector.multiply(1 / targetVector.hypot());
                targetVector.multiply(Math.abs(drivePower));

                motorFL.setPower(targetVector.getX() + targetVector.getY() - turnOutput);
                motorFR.setPower(-targetVector.getX() + targetVector.getY() + turnOutput);
                motorBL.setPower(-targetVector.getX() + targetVector.getY() - turnOutput);
                motorBR.setPower(targetVector.getX() + targetVector.getY() + turnOutput);
            }
        }
    }

    public void waitForDrive(Telemetry telemetry) {
        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < .3 && ThreadUtils.isRunThread()) {
            if (Math.abs(distanceTo(targetPos)) > 2.5 || Math.abs(distanceToAngle(targetAngle)) > Math.PI/45)
                timer.reset();

            if (telemetry != null) {
                telemetry.addData("Coord", this.getCoord());
                telemetry.addData("Target Coord", targetPos);
                telemetry.addData("Heading", getAngle());
                telemetry.addData("Target Heading", targetAngle);
                telemetry.update();
            }
        }
    }

    public void waitForDrive() {
        waitForDrive(null);
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
