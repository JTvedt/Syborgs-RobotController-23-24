package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.Controller;

/**
* TeleOp to test the values of the left and right claw open and close positions
* @author Tyler Philip
* */

@TeleOp(name = "Config Claw")
public class ClawConfig extends LinearOpMode {

    public Servo leftClaw;
    public Servo rightClaw;

    public Servo spinClaw;

    public double leftCloseValue = 0.7;
    public double rightCloseValue = 0.3;
    public double leftOpenValue = 1.0;
    public double rightOpenValue = 0.0;

    public double spinDown = 0.0;
    public double spinBack = 1.0;

    @Override
    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "LC");
        rightClaw = hardwareMap.get(Servo.class, "RC");
        spinClaw = hardwareMap.get(Servo.class,"SC");

        Controller controller = new Controller(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if (controller.pressingButton("X")) { //Close claw based on the close values
                leftClaw.setPosition(leftCloseValue);
                rightClaw.setPosition(rightCloseValue);
            }
            if (controller.pressingButton("Y")) { //Open claw based on the open values
                leftClaw.setPosition(leftOpenValue);
                rightClaw.setPosition(rightOpenValue);
            }
            if(controller.pressingButton("DU")){
                spinClaw.setPosition(spinBack);
            }
            if(controller.pressingButton("DD")){
                spinClaw.setPosition(spinDown);
            }

            if(controller.holdingButton("A")){
                if(controller.pressingButton("RT")){ //Right trigger and holding down A to increase the right close value
                    rightCloseValue += 0.1;
                }
                if(controller.pressingButton("LT")){ //Left trigger and holding down A to increase the left close value
                    leftCloseValue += 0.1;
                }
                if(controller.pressingButton("RB")){ //Right bumper and holding down A to increase the right open value
                    rightOpenValue += 0.1;
                }
                if(controller.pressingButton("LB")){ //Left bumper and holding down A to increase the left open value
                    leftOpenValue += 0.1;
                }
                if(controller.pressingButton("DR")){
                    spinBack += 0.1;
                }
                if(controller.pressingButton("DL")){
                    spinDown += 0.1;
                }
            }

            else if(controller.holdingButton("B")){
                if(controller.pressingButton("RT")){  //Right trigger and holding down B to decrease right close value
                    rightCloseValue -= 0.1;
                }
                if(controller.pressingButton("LT")){ //Left trigger and holding down B to decrease left close value
                    leftCloseValue -= 0.1;
                }
                if(controller.pressingButton("RB")){ //Right bumper and holding down B to decrease right open value
                    rightOpenValue -= 0.1;
                }
                if(controller.pressingButton("LB")){ //Left bumper and holding down B to decrease left open value
                    leftOpenValue -= 0.1;
                }
                if(controller.pressingButton("DR")){
                    spinBack -= 0.1;
                }
                if(controller.pressingButton("DL")){
                    spinDown -= 0.1;
                }
            }

            telemetry.addData("Right Close Position", rightCloseValue);
            telemetry.addData("Left Close Position", leftCloseValue);
            telemetry.addData("Right Open Position", rightOpenValue);
            telemetry.addData("Left Open Position", leftOpenValue);
            telemetry.addData("Current Right Claw Position",rightClaw.getPosition());
            telemetry.addData("Current Left Claw Position",leftClaw.getPosition());

            telemetry.update();


        }

    }
}
