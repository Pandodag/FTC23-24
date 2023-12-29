package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

	public static final int CAMERA_WIDTH = 640;
	public static final int CAMERA_HEIGHT = 360;
	// lens intrinsics
	public static final double FX = 578.272;
	public static final double FY = 578.272;
	public static final double CX = CAMERA_WIDTH / 2.0;
	public static final double CY = CAMERA_HEIGHT / 2.0;

	private Telemetry telemetry;
	private HardwareMap hardwareMap;

	private OpenCvCamera camera = null;

	public Camera(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, CustomOpenCVPipeline pipeline) {
		this.telemetry = telemetry;
		this.hardwareMap = hardwareMap;

		WebcamName webcamName = hardwareMap.get(WebcamName.class, deviceName);
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

		camera.setPipeline(pipeline);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
			}
		});
	}

	public void logData() {
		telemetry.addLine("FPS: " + getFps());
		telemetry.addLine("MAX FPS: " + getCurrentPipelineMaxFps());
		telemetry.addLine("Frame Count: " + getFrameCount());
		telemetry.addLine("Overhead Time Ms: " + getOverheadTimeMs());
		telemetry.addLine("Pipeline Time Ms: " + getPipelineTimeMs());
		telemetry.addLine("Total Frame Time Ms: " + getTotalFrameTimeMs());
		telemetry.update();  // make it appear on driver station
	}

	public float getFps() {
		return camera.getFps();
	}

	public int getCurrentPipelineMaxFps() {
		return camera.getCurrentPipelineMaxFps();
	}

	public int getFrameCount() {
		return camera.getFrameCount();
	}

	public int getOverheadTimeMs() {
		return camera.getOverheadTimeMs();
	}

	public int getPipelineTimeMs() {
		return camera.getPipelineTimeMs();
	}

	public int getTotalFrameTimeMs() {
		return camera.getTotalFrameTimeMs();
	}
}
