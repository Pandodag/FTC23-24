package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ConeDetectionPipeline extends CustomOpenCVPipeline {

	private Mat hsv1 = new Mat();
	private Mat hsv2 = new Mat();
	private Mat combinedFiltered = new Mat();
	// lower boundary: Hue range (0-10)
	private Scalar lowRed1 = new Scalar(0, 100, 20);
	private Scalar highRed1 = new Scalar(10, 255, 255);

	// upper boundary: Hue range (160-179)
	private Scalar lowRed2 = new Scalar(160, 100, 20);
	private Scalar highRed2 = new Scalar(179, 255, 255);

	public ConeDetectionPipeline(Telemetry telemetry) {
		super(telemetry);
	}

	@Override
	public Mat processFrame(Mat input) {
		// convert input to HSV colors
		// range of 0 - 180 for hue
		Imgproc.cvtColor(input, hsv1, Imgproc.COLOR_RGB2HSV);
		Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV);

		// filter the lower boundary with Hue ranges
		// and convert to grayscale
		Core.inRange(hsv1, lowRed1, highRed1, hsv1);
		// filter the upper boundary with Hue ranges
		Core.inRange(hsv2, lowRed2, highRed2, hsv2);

		Core.add(hsv1, hsv2, combinedFiltered);

		Rect boundingBox = getBoundingBox(combinedFiltered);
		if (boundingBox != null) {
			Imgproc.rectangle(input, boundingBox, new Scalar(255, 0, 0));
		}
		return input;
	}

	private Rect getBoundingBox(Mat in) {
		// store list of contours
		List<MatOfPoint> contours = new ArrayList<>();
		Imgproc.findContours(in, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		// check that there are contours
		if (contours.size() == 0) {return null;}

		Rect[] boundingRects = new Rect[contours.size()]; // store all possible bounding rects

		// get the bounding rectangles
		for (int i = 0; i < contours.size(); i++) {
			boundingRects[i] = Imgproc.boundingRect(new MatOfPoint(contours.get(i)));
		}
		// choose the largest bounding rect by area
		Rect max = boundingRects[0];
		for (int i = 1; i < boundingRects.length; i++) {
			if (boundingRects[i].area() > max.area()) {
				max = boundingRects[i];
			}
		}
		return max;
	}
}
