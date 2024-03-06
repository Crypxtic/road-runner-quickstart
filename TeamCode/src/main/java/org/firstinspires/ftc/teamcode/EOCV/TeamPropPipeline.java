package org.firstinspires.ftc.teamcode.EOCV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TeamPropPipeline implements VisionProcessor {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat mat2 = new Mat();
    public int finalPosition;

    public TeamPropPipeline(Telemetry t){
        telemetry = t;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public enum position {
        LEFT,
        MIDDLE,
        RIGHT
    }


    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        List<Rect> RectList = new ArrayList<>();
        List<MatOfPoint> contourList = new ArrayList<MatOfPoint>();
        MatOfPoint2f mop2f;
        List<MatOfPoint> boxContours = new ArrayList<>();
        position myPosition = position.LEFT;

        Imgproc.GaussianBlur(input, input, new Size(7, 7), 7);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, mat2, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 80, 50);
        Scalar highHSV = new Scalar(15,255,255);
        Scalar lowHSV2 = new Scalar(165, 80, 50);
        Scalar highHSV2 = new Scalar(180,255,255);


        MatOfPoint approxf1 = new MatOfPoint();
        List<MatOfPoint> approxList = new ArrayList<>();

        Core.inRange(mat, lowHSV, highHSV, mat);
        Core.inRange(mat2, lowHSV2, highHSV2, mat2);

        Core.bitwise_or(mat, mat2, mat);

        Mat hierarchey = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mat, contours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        mat.release();
        mat2.release();

        for (MatOfPoint contour : contours) {

            /* approx poly dp

            MatOfPoint2f c2f = new MatOfPoint2f(contour.toArray());
            double peri = Imgproc.arcLength(c2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(c2f, approx, 0.001 * peri, true);
            approx.convertTo(approxf1, CvType.CV_32S);
            approxList.add(approxf1);
            Imgproc.drawContours(input, approxList, 0, new Scalar(0, 255, 0));
            */


            Rect rectangle = Imgproc.boundingRect(contour);
            Imgproc.rectangle(input, rectangle, new Scalar(0, 255, 0));
            double greater = Math.max(rectangle.height, rectangle.width);
            double lesser = Math.min(rectangle.height, rectangle.width);
            if (greater < 100 || lesser < 10) {
                continue;
            }
            RectList.add(rectangle);
            contourList.add(contour);

        }
        boolean gotDetect = false;
        for (int i = 0; i < RectList.size(); i++) {
            Rect r = RectList.get(i);
            mop2f = new MatOfPoint2f(contourList.get(i).toArray()); //get best contour of contourlist
            RotatedRect rotatedRect = Imgproc.minAreaRect(mop2f);
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            double width = Math.min(rotatedRect.size.width, rotatedRect.size.height);
            if (width > 50){
                boxContours.add(new MatOfPoint(vertices));
                if (rotatedRect.center.x < 320) {
                    myPosition = position.LEFT;
                    finalPosition = 1;
                }
                else {
                    myPosition = position.MIDDLE;
                    finalPosition = 2;
                }
                gotDetect = true;
            }
            if (!gotDetect) {
                myPosition = position.RIGHT;
                finalPosition = 3;
            }
            //telemetry.addData("width", width);
//            telemetry.addData("center", rotatedRect.center.x);
        }
        telemetry.addData("Contours", contourList.size());
        Imgproc.drawContours(input, boxContours, -1, new Scalar(128, 255,255), 1);
        telemetry.addData("Place", myPosition.toString());
        telemetry.addData("Place number", finalPosition);
        return input;
    }

}
