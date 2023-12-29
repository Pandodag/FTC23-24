package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline;

@Autonomous
public class TestOpenCV extends OpMode {

    private Camera camera;

    @Override
    public void init() {
        camera = new Camera(telemetry, hardwareMap, "webcam1", new ConeDetectionPipeline(telemetry));
    }

    @Override
    public void loop() {

    }
}