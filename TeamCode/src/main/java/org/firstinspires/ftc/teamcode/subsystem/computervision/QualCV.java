package org.firstinspires.ftc.teamcode.subsystem.computervision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

public class QualCV implements ComputerVision {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private OpenCvCamera camera;

    public QualCV(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public int getSpikeMark() {
        AprilTagDetection tag;
        for (AprilTagDetection atd : getDetections()) {
            if (atd.id == 11) {
                tag = atd;
                break;
            }
        }

        return 0;
    }

    @Override
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
}
