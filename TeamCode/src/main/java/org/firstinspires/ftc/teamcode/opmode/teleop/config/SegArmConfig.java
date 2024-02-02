package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystem.arm.SegmentedArm;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

@TeleOp(name="Config Seg Arm")
public class SegArmConfig extends OpMode {
    private SegmentedArm arm;
    private Controller controller;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        arm = new SegmentedArm(hardwareMap);
    }

    @Override
    public void loop() {
        double deltaArm;
        if (controller.holdingButton("RT"))
            deltaArm = Math.PI/2;
        else
            deltaArm = Math.PI/6;

        if (controller.pressingButton("Y"))
            arm.changeUpperArm(deltaArm);
        if (controller.pressingButton("X"))
            arm.changeUpperArm(-deltaArm);

        if (controller.holdingButton("B"))
            arm.changeUpperArm(Math.PI/180);
        if (controller.holdingButton("A"))
            arm.changeUpperArm(-Math.PI/180);


        double deltaServo;
        if (controller.holdingButton("RT"))
            deltaServo = Math.PI/2;
        else
            deltaServo = Math.PI/6;

        if (controller.pressingButton("DU"))
            arm.changeForearm(deltaServo);
        if (controller.pressingButton("DL"))
             arm.changeForearm(-deltaServo);

        if (controller.holdingButton("DR"))
            arm.changeForearm(Math.PI/180);
        if (controller.holdingButton("DD"))
            arm.changeForearm(-Math.PI/180);

        if (controller.pressingButton("LB"))
            arm.setForearm(Math.PI);
        if (controller.pressingButton("RB"))
            arm.initialExtension();

        if (controller.holdingButton("LT"))
            arm.tempFlag = true;
        else
            arm.tempFlag = false;

        telemetry.addData("Upper Target", arm.getTargetPosition());
        telemetry.addData("Upper Actual", arm.getCurrentPosition());
        telemetry.addData("Upper Angle", arm.getUpperArmAngle());
        telemetry.addData("Upper Power", arm.powerTemp);
        telemetry.addData("Upper Power Actual", arm.armMotor.getPower());
        telemetry.addData("Forearm Target", arm.getForeTarget());
        telemetry.addData("Forearm Actual", arm.getForeCurrent());
        telemetry.addData("Forearm Angle", arm.getForearmAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        ThreadUtils.stopThreads();
    }
}
