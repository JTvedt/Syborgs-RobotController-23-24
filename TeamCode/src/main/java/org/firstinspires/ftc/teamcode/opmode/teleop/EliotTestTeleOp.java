package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.Controller;

/**
 * Teleop made for testing purposes
 * @author Eliot Bih and Tyler Philip
 * */

@TeleOp(name = "[TEST] Eliot TeleOp")
public class EliotTestTeleOp extends LinearOpMode {

    public void runOpMode() {
        TeleOpDrive chassis = new TeleOpDrive(hardwareMap);
        Controller controller = new Controller(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            chassis.teleDrive(controller.getValue("LX"),controller.getValue("LY"), controller.getValue("RX"),telemetry);
        }
    }

}
