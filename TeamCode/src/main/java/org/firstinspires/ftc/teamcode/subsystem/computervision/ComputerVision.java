package org.firstinspires.ftc.teamcode.subsystem.computervision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;
public interface ComputerVision {
    int getSpikeMark();
    List<AprilTagDetection> getDetections();
}
