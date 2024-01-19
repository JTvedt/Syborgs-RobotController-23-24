package org.firstinspires.ftc.teamcode.subsystem.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SegmentedArm extends ArmImpl {
    private Servo armServo;

    public SegmentedArm(HardwareMap hardwareMap) {
        super(hardwareMap);

        armServo = hardwareMap.get(Servo.class, "AS");
    }

    public void setCoordinate(int x, int y) {

    }
}
