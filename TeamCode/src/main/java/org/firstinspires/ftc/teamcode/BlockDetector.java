package org.firstinspires.ftc.teamcode;
import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.List;

public class BlockDetector extends OpenCVPipeline{

    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        Mat hsv = rgba;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Mat threshold = gray;
        Core.inRange(hsv, new Scalar(6, 154, 94), new Scalar(59, 255, 255), threshold);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        if(contours.size()>0) {
            double maxVal = 0;
            int maxValIdx = -1;
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx; 
                }
            }

            Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_HSV2RGB);
            Imgproc.rectangle(rgba, boundingRect.tl(),boundingRect.br(),new Scalar (255, 0, 0), 3);
        }
        return rgba;
    }

}
