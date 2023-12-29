/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
public class FinalAutonomous extends LinearOpMode {

	private Camera camera;
	private AprilTagDetectionPipeline aprilTagDetectionPipeline;

	// motors
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;

	private Rect rect1 = new Rect(0, 0, Camera.CAMERA_WIDTH / 3, Camera.CAMERA_HEIGHT);
	private Rect rect2 = new Rect(rect1.x, 0, Camera.CAMERA_WIDTH * 2 / 3, Camera.CAMERA_HEIGHT);
	private Rect rect3 = new Rect(rect2.x, 0, Camera.CAMERA_WIDTH / 3, Camera.CAMERA_HEIGHT);

	private enum Location {
		LEFT, CENTER, RIGHT;
	}

	@Override
	public void runOpMode() {
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(telemetry, Camera.FX, Camera.FY, Camera.CX, Camera.CY);
		camera = new Camera(telemetry, hardwareMap, "webcam1", aprilTagDetectionPipeline);

		leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
		rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

		waitForStart();
		telemetry.setMsTransmissionInterval(50);

		int numFramesWithoutDetection = 0;
		final float DECIMATION_HIGH = 3;
		final float DECIMATION_LOW = 2;
		final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
		final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

		final double FEET_PER_METER = 3.28084;

		Location location = null;

		while (opModeIsActive()) {
			// if we're still finding the position to go to
			if (location == null) {
				Point point = aprilTagDetectionPipeline.updateInLoop();
				if (point != null) {
					if (rect1.contains(point)) { location = Location.LEFT; }
					if (rect2.contains(point)) { location = Location.CENTER; }
					if (rect3.contains(point)) { location = Location.RIGHT; }
				}
			}
			telemetry.update();
			sleep(20);
		}
	}
}