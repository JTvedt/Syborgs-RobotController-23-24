package org.firstinspires.ftc.teamcode.subsystem.computervision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TylerPipeline extends OpenCvPipeline {
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution

    private double pipeline;
    private int color;
    private String position;
    private double cX;
    private double cY;

    final int BLUE = 1;
    final int RED = 2;

    private boolean flag = false;
    private double width;

    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 2;  // Replace with the actual width of the object in real-world units
    public static final double FOCAL_LENGTH = 728;  // Replace with the focal length of the camera in pixels

    public TylerPipeline(int color) {
        super();
        this.color = color;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Find contours of the detected red regions
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(preprocessFrame(input), contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest red contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";

            if((int) cX > (426)){
                position = "Right";
            }
            else if((int) cX < (213)){
                position = "Left";
            }
            else{
                position = "Middle";
            }

            Imgproc.putText(input, position, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);


            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Scalar lowerColor;
        Scalar upperColor;
        lowerColor = new Scalar(80, 100, 50);
        upperColor = new Scalar(160, 255, 255);


        Mat redMask = new Mat();
        Core.inRange(hsvFrame, lowerColor, upperColor, redMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

        return redMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }
    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }
    private double getDistance(double width) {
        double distance = (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * FOCAL_LENGTH) / width;
        return distance;
    }
    public String getPosition() {
        return position;
    }
}
