package org.firstinspires.ftc.teamcode.opmode.teleop.config;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.arm.ActuatorArm;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Actuator Arm")
public class ActuatorArmConfig extends BaseOpMode {
    @Override
    public void init() {
        super.init();

        arm = new ActuatorArm(hardwareMap);
    }

    @Override
    public void loop() {
        if (p1.pressingButton("A"))
            arm.setExtension(5);
        if (p1.pressingButton("B"))
            arm.setExtension(0);
        if (p1.pressingButton("X"))
            arm.setArm(Math.PI/2);
        if (p1.pressingButton("Y"))
            arm.setArm(0);
        if (p1.pressingButton("LT"))
            arm.setWrist(0);

        telemetry.addData("Arm Target", arm.getTargetPosition());
        telemetry.addData("Arm Current", arm.getCurrentPosition());
        telemetry.addData("Arm Power", arm.armMotor.getPower());
        telemetry.addData("Extension Target", arm.getExtensionTarget());
        telemetry.addData("Extension Current", arm.actuator.getCurrentPosition());
        telemetry.addData("Actuator Power", arm.actuator.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
