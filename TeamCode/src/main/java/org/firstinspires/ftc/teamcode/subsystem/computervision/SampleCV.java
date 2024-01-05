package org.firstinspires.ftc.teamcode.subsystem.computervision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public class SampleCV implements ComputerVision {
    private HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private OpenCvCamera camera;
    private SampleCVProcessor processor;

    public SampleCV(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        processor = new SampleCVProcessor();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .addProcessor(aprilTag)
                .addProcessor(processor)
                .build();
    }

    @Override
    public int getSpikeMark() {
        return processor.getSpikeMark();
    }

    @Override
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
}