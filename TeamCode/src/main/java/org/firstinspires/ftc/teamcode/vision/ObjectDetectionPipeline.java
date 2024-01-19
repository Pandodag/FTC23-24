package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class ObjectDetectionPipeline extends CustomOpenCVPipeline {

    private Rect rect1 = new Rect(0, 6, 5, 5);
    private Rect rect2 = new Rect(15, 3, 7, 6);
    private Rect rect3 = new Rect(27, 3, 6, 8);

//    private Scalar color = new Scalar(0, 59, 217);
    private Scalar color = new Scalar(218, 42, 17);

    Mat zone1, zone2, zone3;

    public static enum Location {
        LEFT, CENTER, RIGHT, NULL
    }

    private Location location = Location.NULL;

    public ObjectDetectionPipeline(Telemetry telemetry) {
        super(telemetry);
        change(rect1);
        change(rect2);
        change(rect3);
    }

    private void change(Rect r) {
        r.x = (int) (r.x * Camera.CAMERA_WIDTH / 33.0);
        r.y = (int) (r.y * Camera.CAMERA_HEIGHT / 19.0);
        r.width = (int) (r.width * Camera.CAMERA_WIDTH / 33.0);
        r.height = (int) (r.height * Camera.CAMERA_HEIGHT / 19.0);
    }

    public double d1, d2, d3, actualD;
    public Scalar a1 = new Scalar(0,0,0), a2 = new Scalar(0, 0, 0), a3 = new Scalar(0, 0, 0);
    @Override
    public Mat processFrame(Mat input) {
//        return input;
        zone1 = input.submat(rect1);
        zone2 = input.submat(rect2);
        zone3 = input.submat(rect3);

        Scalar avgColor1 = Core.mean(zone1);
        Scalar avgColor2 = Core.mean(zone2);
        Scalar avgColor3 = Core.mean(zone3);

        a1 = avgColor1;
        a2 = avgColor2;
        a3 = avgColor3;

        // for camera
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);
        d1 = colorDistance(avgColor1, color);
        d2 = colorDistance(avgColor2, color);
        d3 = colorDistance(avgColor3, color);
        actualD = Math.min(Math.min(d1, d2), d3);
        if (actualD == d1) {
            location = Location.LEFT;
        } else if (actualD == d2) {
            location = Location.CENTER;
        } else {
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
