package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;

@TeleOp(name="Config Arm")
public class ArmConfig extends LinearOpMode {
    public DcMotor armMotor;

    @Override
    public void runOpMode(){
        Controller controller = new Controller(gamepad1);

        armMotor = hardwareMap.get(DcMotor.class, "AM");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            if(controller.pressingButton("A")) {
                armMotor.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Direction", "Reverse");
            }
            else if(controller.pressingButton("B")) {
                armMotor.setDirection(DcMotor.Direction.FORWARD);
                telemetry.addData("Direction", "Forward");
            }
            armMotor.setPower(controller.getValue("LY"));

            telemetry.addData("Current Position",armMotor.getCurrentPosition());
            telemetry.addData("Motor Power", armMotor.getPower());
            telemetry.update();
        }
    }
}
