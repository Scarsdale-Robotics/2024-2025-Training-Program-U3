package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PipelineThatSharesInformationAsVisionProcessor implements VisionProcessor {

    // based off https://scarsdale-robotics.gitbook.io/the-wiki/programming/opencv-tutorial-and-resources/7.-contour-detection
    public Scalar upper = new Scalar(17, 255, 255);
    public Scalar lower = new Scalar(4, 116, 80);

    Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(4, 4));

    Mat result = new Mat();

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();

    public Scalar contColor = new Scalar(255, 0, 0);

    private int detectedPixels = 0;
    private Point largestContourBoundingBoxCenter = new Point();  // largest contour determined by internal area


    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, result, Imgproc.COLOR_RGB2HSV);
        Core.inRange(result, lower, upper, result);


        // addition: since we want to share the number of detected pixels to other files,
        //           we will store the number of detected pixels when processing this frame.
        //           the following code counts the number of detected pixels
        int _detectedPixels = 0;
        for (int y = 0; y < result.height(); y++) {
            for (int x = 0; x < result.width(); x++) {
                if (Arrays.equals(result.get(y, x), new double[]{255})) {  // if pixel is detected
                    _detectedPixels++;
                }
            }
        }
        detectedPixels = _detectedPixels;


        Imgproc.morphologyEx(result, result, Imgproc.MORPH_OPEN, kernel);

        // result: the input image
        // contours: the list to store the contours in
        // hierarchy: the mat to store information about which contours are contained by others
        // Imgproc.RETR_EXTERNAL: tells OpenCV to only return the outermost contours
        // Imgproc.CHAIN_APPROX_SIMPLE: tells OpenCV to return approximate contours to save memory
        Imgproc.findContours(result, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        // addition: since we want to share the center of the bounding box of the largest contour,
        //           we will store this center point when processing this frame.
        MatOfPoint largestContour = getLargestContour();
        Rect largestContourBoundingBox = Imgproc.boundingRect(largestContour.t());
        largestContourBoundingBoxCenter = new Point(
                largestContourBoundingBox.x + largestContourBoundingBox.width / 2.0,
                largestContourBoundingBox.y + largestContourBoundingBox.height / 2.0
        );


        // convert to RGB so we can draw with colors
        Imgproc.cvtColor(result, result, Imgproc.COLOR_GRAY2RGB);

        // result: the image to draw on
        // contours: the contours to draw
        // -1: the index of the contour to draw. -1 means draw all of them.
        // contColor: the color to draw the contours in
        Imgproc.drawContours(result, contours, -1, contColor);

        // clear list so it can be reused next frame
        contours.clear();

        return result;
    }

    private MatOfPoint getLargestContour() {
        MatOfPoint largestContour = null; double largestArea = Integer.MIN_VALUE;

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);

            if (contourArea > largestArea) {
                largestArea = contourArea;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    // other code (such as OpModes) can now call the following methods to obtain realtime about the frame!
    public int getDetectedPixels() {
        return detectedPixels;
    }

    public Point getLargestContourBoundingBoxCenter() {
        return largestContourBoundingBoxCenter;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // ignore for now
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // ignore for now
    }
}
