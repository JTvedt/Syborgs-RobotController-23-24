package org.firstinspires.ftc.teamcodev2.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcodev2.util.ThreadUtils;
import org.firstinspires.ftc.teamcodev2.util.math.DistanceUnit;
import org.firstinspires.ftc.teamcodev2.util.math.MathUtils;

public class AutonomousDrivetrainImpl extends DrivetrainImpl implements AutonomousDrivetrain {
    public static final double LOW_POWER = 0.1;
    public static final double HIGH_POWER = 0.8;

    public static final int LOW_THRESHOLD = DistanceUnit.toTicks(1, DistanceUnit.CM);
    public static final int HIGH_THRESHOLD = DistanceUnit.toTicks(8, DistanceUnit.CM);

    private double distanceUnit = DistanceUnit.CM;

    public AutonomousDrivetrainImpl(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public double getDistanceUnit() {
        return distanceUnit;
    }

    public void setDistanceUnit(double distanceUnit) {
        this.distanceUnit = distanceUnit;
    }

    @Override
    public void polarMove(double distance, double angle) {
        cartesianMove(Math.cos(angle) * distance, Math.sin(angle) * distance);
    }

    @Override
    public void cartesianMove(double x, double y) {
        int driveTicks = DistanceUnit.toTicks(x, distanceUnit);
        int strafeTicks = DistanceUnit.toTicks(y, distanceUnit);

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition(driveTicks + strafeTicks);
        motorFR.setTargetPosition(driveTicks - strafeTicks);
        motorBL.setTargetPosition(driveTicks - strafeTicks);
        motorBR.setTargetPosition(driveTicks + strafeTicks);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (isMoving()) {
            setPower(MathUtils.normalize(distanceToTarget(), LOW_THRESHOLD, HIGH_THRESHOLD, LOW_POWER, HIGH_POWER));
        }

        ThreadUtils.rest();
    }

    public double distanceToTarget() {
        return motorList.stream().mapToDouble(motor -> motor.getTargetPosition() - motor.getCurrentPosition()).sum();
    }

    public double distanceToAngle(double angle) {
        return MathUtils.normalizeAngle(getAngle() - angle);
    }

    @Override
    public void strafe(double distance) {
        cartesianMove(distance, 0);
    }

    @Override
    public void drive(double distance) {
        cartesianMove(0, distance);
    }

    @Override
    public void spinTo(double angle) {
        while (Math.abs(distanceToAngle(angle)) > Math.PI/180) {
            double power = Math.signum(distanceToAngle(angle)) * MathUtils.normalize(distanceToAngle(angle), Math.PI/36, 18 * Math.PI/36, 0.1, 1.0);

            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
        }

        ThreadUtils.rest();
    }

    @Override
    public void spin(double angle) {

    }

    @Override
    public double getAngle() {
        return 0;
    }
}