package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.controller.Controller;
/**
 * TeleOp for configuring the intake which consists of a servo and a motor
 * @author Tyler Philip
 * */
@TeleOp(name="Config Intake")
public class IntakeConfig extends LinearOpMode {
    public DcMotor intakeLift;
    public CRServo intakeSpin;
    @Override
    public void runOpMode(){
        Controller controller = new Controller(gamepad1);

        intakeLift = hardwareMap.get(DcMotor.class,"IM");
        intakeSpin = hardwareMap.get(CRServo.class,"IS");

        boolean spinning = false;

        waitForStart();

        while(opModeIsActive()){

            //Allows the user to reverse the direction of the motors for the arm
            if(controller.pressingButton("A")){
                intakeLift.setDirection(DcMotor.Direction.REVERSE);
                telemetry.addData("Intake Direction:", "Reverse");
            }
            else if(controller.pressingButton("B")){
                intakeLift.setDirection(DcMotor.Direction.FORWARD);
                telemetry.addData("Intake Direction:", "Forward");
            }
            if(controller.pressingButton("RT")){
                intakeSpin.setDirection(CRServo.Direction.REVERSE);
                telemetry.addData("Spin Direction:", "Reverse");
            }
            if(controller.pressingButton("LT")){
                intakeSpin.setDirection(CRServo.Direction.FORWARD);
                telemetry.addData("Spin Direction:", "Forward");
            }


            if(controller.pressingButton("X")){spinning = true;}

            else if(controller.pressingButton("Y")){spinning=false;}

            if(spinning)
                intakeSpin.setPower(1);
            else
                intakeSpin.setPower(0);

            intakeLift.setPower(controller.getValue("RY"));

            telemetry.addData("Spinning:",spinning);

            telemetry.update();
        }
    }
}
