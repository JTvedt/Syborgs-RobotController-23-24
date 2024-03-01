package org.firstinspires.ftc.teamcode.subsystem.computervision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class TeamPropProcessor implements VisionProcessor {
    Rect LEFT_RECTANGLE;
    Rect MIDDLE_RECTANGLE;
    Rect RIGHT_RECTANGLE;

    Mat hsvMat = new Mat();
    Mat lowMat = new Mat();
    Mat highMat = new Mat();
    Mat detectedMat = new Mat();

    Scalar lowerRedThresholdLow = new Scalar(0,125,125);
    Scalar getLowerRedThresholdHigh = new Scalar(10,255,255);

    Scalar upperRedThresholdLow = new Scalar(165,125,125);
    Scalar upperRedThresholdHigh = new Scalar(180,255, 255);
    double leftThreshold = 0.1;
    double middleThreshold = 0.1;
    double rightThreshold = 0.1;

    String objPos = "";
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        LEFT_RECTANGLE = new Rect(
                new Point(0,0),
                new Point(0.33 * width, height)
        );
        MIDDLE_RECTANGLE = new Rect(
                new Point(0.33 * width,0),
                new Point(0.66 * width, height)
        );
        RIGHT_RECTANGLE = new Rect(
                new Point(0.66 * width,0),
                new Point(width,height)
        );
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        //0-20, 34-360
        //50%, 50%

        Core.inRange(hsvMat, lowerRedThresholdLow, getLowerRedThresholdHigh, lowMat);
        Core.inRange(hsvMat, upperRedThresholdLow, upperRedThresholdHigh, highMat);

        Core.bitwise_or(lowMat,highMat,detectedMat);

        double leftPercent = (Core.sumElems(detectedMat.submat(LEFT_RECTANGLE)).val[0] / 255) / LEFT_RECTANGLE.area();
        double middlePercent = (Core.sumElems(detectedMat.submat(MIDDLE_RECTANGLE)).val[0] / 255) / MIDDLE_RECTANGLE.area();
        double rightPercent = (Core.sumElems(detectedMat.submat(RIGHT_RECTANGLE)).val[0] / 255) / RIGHT_RECTANGLE.area();

        Scalar redBorder = new Scalar(255,0,0);
        Scalar greenBorder = new Scalar(0,255,0);

        if(leftPercent > middlePercent && leftPercent > rightPercent && leftPercent > leftThreshold)
        {
            //Left is the highest percentage
            objPos = "Left";
            Imgproc.rectangle(frame,LEFT_RECTANGLE,greenBorder);
            Imgproc.rectangle(frame,MIDDLE_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,RIGHT_RECTANGLE,redBorder);


        }
        else if(middlePercent > leftPercent && middlePercent > rightPercent && middlePercent > middleThreshold)
        {
            //Middle is the highest percentage
            objPos = "Middle";
            Imgproc.rectangle(frame,LEFT_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,MIDDLE_RECTANGLE,greenBorder);
            Imgproc.rectangle(frame,RIGHT_RECTANGLE,redBorder);

        }
        else if(rightPercent > middlePercent && rightPercent > leftPercent &&  rightPercent > rightThreshold)
        {
            //Right is the highest percentage
            objPos = "Right";
            Imgproc.rectangle(frame,LEFT_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,MIDDLE_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,RIGHT_RECTANGLE,greenBorder);

        }
        else
        {
            //Team prop was not detected
            objPos = "Middle"; //Sets to middle as a guess
            Imgproc.rectangle(frame,LEFT_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,MIDDLE_RECTANGLE,redBorder);
            Imgproc.rectangle(frame,RIGHT_RECTANGLE,redBorder);


        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public String getObjPos()
    {
        return objPos;
    }
}
