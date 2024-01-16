package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class ObjectDetectionPipeline extends CustomOpenCVPipeline {

    private Rect rect1 = new Rect();
    private Rect rect2 = new Rect();
    private Rect rect3 = new Rect();

    private Scalar color = new Scalar(0, 0, 255);

    Mat zone1, zone2, zone3;

    public static enum Location {
        LEFT, CENTER, RIGHT, NULL
    }

    private Location location = Location.NULL;

    public ObjectDetectionPipeline(Telemetry telemetry) {
        super(telemetry);
    }

    @Override
    public Mat processFrame(Mat input) {
        zone1 = input.submat(rect1);
        zone2 = input.submat(rect2);
        zone3 = input.submat(rect3);

        Scalar avgColor1 = Core.mean(zone1);
        Scalar avgColor2 = Core.mean(zone2);
        Scalar avgColor3 = Core.mean(zone3);

        // for camera
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);
        double d1 = colorDistance(avgColor1, color);
        double d2 = colorDistance(avgColor2, color);
        double d3 = colorDistance(avgColor3, color);
        if (d1 < d2 && d1 < d3) {
            location = Location.LEFT;
        } else if (d2 < d1 && d2 < d3) {
            location = Location.CENTER;
        } else if (d3 < d1 && d3 < d2) {
            location = Location.RIGHT;
        }
        return input;
    }

    public Location getLocation() {
        return location;
    }

    private double colorDistance(Scalar a, Scalar b) {
        double x = a.val[0] - b.val[0];
        double y = a.val[1] - b.val[1];
        double z = a.val[2] - b.val[2];
        return Math.sqrt(x * x + y * y + z * z);
    }
}
