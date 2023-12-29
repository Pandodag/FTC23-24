package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class CustomOpenCVPipeline extends OpenCvPipeline {
	protected Telemetry telemetry;

	public CustomOpenCVPipeline(Telemetry telemetry) {
		this.telemetry = telemetry;
	}
}
