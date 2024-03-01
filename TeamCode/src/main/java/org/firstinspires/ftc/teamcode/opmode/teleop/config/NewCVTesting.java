package org.firstinspires.ftc.teamcode.opmode.teleop.config;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.computervision.TeamPropProcessor;
import org.firstinspires.ftc.teamcode.subsystem.computervision.TylerCV;

@TeleOp(name="Testing CV")
public class NewCVTesting extends OpMode {


    private String position = "";
    @Override
    public void init() {
        //TylerCV cv = new TylerCV(hardwareMap,2); //Blue
        TylerCV cv = new TylerCV(hardwareMap,1); //Red
        position = cv.getPosition();
    }

    @Override
    public void loop() {
        telemetry.addData("Position", position);
    }
}
