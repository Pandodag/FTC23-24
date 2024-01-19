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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MotorUtil;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.vision.ObjectDetectionPipeline;
import org.opencv.core.Rect;

@Autonomous
public class FinalAutonomousFar extends LinearOpMode {

	private Camera camera;
	private ObjectDetectionPipeline detectionPipeline;

	// motors
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;

	private Servo armServo = null;
	private Servo elbowServo = null;
	private Servo wristServo = null;
	private Servo handServo = null;

	private ElapsedTime timer;

	// park far left
	private void parkFarLeft() {
		// move left
		if (timer.time() < 3.4) {
			MotorUtil.strafeBot(
					leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, -0.6);
		} else {
			leftFrontDrive.setPower(0);
			rightFrontDrive.setPower(0);
			leftBackDrive.setPower(0);
			rightBackDrive.setPower(0);
		}
	}

	/**
	 * Moves with the given axial, lateral for the given duration
	 * @return if task is completed
	 * */
	private boolean move(double axial, double lateral, double duration) {
		if (timer.time() < duration) {
			MotorUtil.moveBot(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, axial, lateral, 0.0);
			return false;
		}
		leftFrontDrive.setPower(0);
		rightFrontDrive.setPower(0);
		leftBackDrive.setPower(0);
		rightBackDrive.setPower(0);
		return true;
	}

	@Override
	public void runOpMode() {
		timer = new ElapsedTime();
		detectionPipeline = new ObjectDetectionPipeline(telemetry);
		camera = new Camera(telemetry, hardwareMap, "webcam1", detectionPipeline);

		leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
		rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

		armServo = hardwareMap.get(Servo.class, "armServo");
		elbowServo = hardwareMap.get(Servo.class, "elbowServo");
		wristServo = hardwareMap.get(Servo.class, "wristServo");
		handServo = hardwareMap.get(Servo.class, "handServo");

		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

		waitForStart();
		telemetry.setMsTransmissionInterval(50);

		timer.reset();

		ObjectDetectionPipeline.Location location = ObjectDetectionPipeline.Location.NULL;

		while (opModeIsActive()) {
			// do detection stuff
			// if the location is still not found
			if (location == ObjectDetectionPipeline.Location.NULL) {
//			if (true) {
				location = detectionPipeline.getLocation();
				timer.reset();
			} else {
				boolean finished = false;
				if (location == ObjectDetectionPipeline.Location.LEFT) {
					finished = move(0.0, -0.8, 0.39);
				} else if (location == ObjectDetectionPipeline.Location.CENTER) {
					finished = move(0.8, 0.0, 0.19);
				} else if (location == ObjectDetectionPipeline.Location.RIGHT) {
					finished = move(0.0, 0.8, 0.46);
				}
			}
			telemetry.addLine("d1: " + detectionPipeline.d1 + " d2: " + detectionPipeline.d2 + " d3: " + detectionPipeline.d3);
			telemetry.addLine("" + detectionPipeline.a1.toString());
			telemetry.addLine("" + detectionPipeline.a2.toString());
			telemetry.addLine("" + detectionPipeline.a3.toString());
			telemetry.addLine("actual d " + detectionPipeline.actualD);

			telemetry.addLine(location.name());
			sleep(20);
//			moveLeft();
			telemetry.update();
		}
	}
}