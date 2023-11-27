package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drivetrain.SampleDrive;

@TeleOp(name = "[TESTING] Eliot Test TeleOp")
public class EliotTestTeleOp extends LinearOpMode {

    SampleDrive drive = new SampleDrive(hardwareMap);

    // THIS WILL NOT WORK ! you only update the stick variables once, so it's not responsive to player input
    // -Jeffrey
    double LStickX = gamepad1.left_stick_x;
    double LStickY = -gamepad1.left_stick_y;
    //reminder that the left_stick_y float corresponds as
    // -1 = up and 1 = down, so I just reversed them here
    // cuz I'm totally gonna forget later
    double RStickX = gamepad1.right_stick_x;

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            // DPad inputs
            // Avoid doing this, these methods are for autonomous and this may have unexpected behaviour
            // -Jeffrey
            if (gamepad1.dpad_up) {
                drive.drive(5);
            }
            if (gamepad1.dpad_down) {
                drive.drive(-5);
            }
            if (gamepad1.dpad_right) {
                drive.strafe(5);
            }
            if (gamepad1.dpad_left) {
                drive.strafe(-5);
            }

            // Stick drive inputs
            drive.cartesianMove(5*LStickX, 5*LStickY);
            drive.spin(0.2*RStickX);
            // TODO: figure out proper coefficients to all these inputs
            // i'm gonna be honest i have no clue if this works
            // and/or if this is drivable but i can't test this rn
            // so ima hope i understand the sample code lmao

        }
    }

}
