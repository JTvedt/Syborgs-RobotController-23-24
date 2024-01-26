package org.firstinspires.ftc.teamcode.subsystem.pipeline;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class spikePipeline extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat middleCrop;
    Mat rightCrop;

    double leftavgfin;
    double middleavgfin;
    double rightavgfin;
    Mat outPut = new Mat();
    Scalar currentColour = new Scalar(255.0,0.0,.0);
    Scalar blankColour = new Scalar(0.0,0.0,255.0);
    int channel;

    Telemetry telemetry;
    String spike = "Unknown";

    public spikePipeline(int colour , Telemetry telem) {
        channel = colour;
        telemetry = telem;

    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("Pipeline Running");
        telemetry.addData("Running?","YES");
        telemetry.update();

        Rect leftRect = new Rect(25,250,50,200);
        Rect middleRect = new Rect(200,225,200,50);
        Rect rightRect = new Rect(550,200,50,200);

        input.copyTo(outPut);
        Imgproc.rectangle(outPut,leftRect,blankColour,2);
        Imgproc.rectangle(outPut,middleRect,blankColour,2);
        Imgproc.rectangle(outPut,rightRect,blankColour,2);

        leftCrop = YCbCr.submat(leftRect);
        middleCrop = YCbCr.submat(middleRect);
        rightCrop = YCbCr.submat(rightRect);


        Core.extractChannel(leftCrop,leftCrop,channel);
        Core.extractChannel(rightCrop,rightCrop,channel);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];

        if((rightavgfin > leftavgfin) && (rightavgfin > middleavgfin)){
            spike = "Right";
            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,blankColour,2);
            Imgproc.rectangle(outPut,middleRect,currentColour,2);
            Imgproc.rectangle(outPut,rightRect,blankColour,2);
        }
        else if((leftavgfin > rightavgfin) && (leftavgfin > middleavgfin)){
            spike = "Left";
            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,currentColour,2);
            Imgproc.rectangle(outPut,middleRect,blankColour,2);
            Imgproc.rectangle(outPut,rightRect,blankColour,2);

        }
        else{
            spike = "Middle";
            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,currentColour,2);;
            Imgproc.rectangle(outPut,middleRect,currentColour,2);
            Imgproc.rectangle(outPut,rightRect,blankColour,2);
        }
        telemetry.addData("RAF",rightavgfin);
        telemetry.addData("MAF",middleavgfin);
        telemetry.addData("LAF",middleavgfin);
        telemetry.update();
        return (outPut);
    }
    public String getSpike(){
        return spike;
    }
}