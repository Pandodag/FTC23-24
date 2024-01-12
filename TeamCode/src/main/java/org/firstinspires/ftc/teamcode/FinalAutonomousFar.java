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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mech.ArmSystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.opencv.core.Point;
import org.opencv.core.Rect;

@Autonomous
public class FinalAutonomousFar extends LinearOpMode {

	private Camera camera;
	private AprilTagDetectionPipeline aprilTagDetectionPipeline;

	// motors
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;

	private Rect rect1 = new Rect(0, 0, (int) (Camera.CAMERA_WIDTH * 6.5 / 33.0), Camera.CAMERA_HEIGHT);
	private Rect rect2 = new Rect(rect1.x, 0, (int) (Camera.CAMERA_WIDTH * 21.0 / 33.0), Camera.CAMERA_HEIGHT);
	private Rect rect3 = new Rect(rect2.x, 0, (int) (Camera.CAMERA_WIDTH * 5.5 / 33.0), Camera.CAMERA_HEIGHT);

	private enum Location {
		LEFT, CENTER, RIGHT;
	}

	private Servo armServo = null;
	private Servo elbowServo = null;
	private Servo wristServo = null;
	private Servo handServo = null;
	private DcMotor extensionMotor = null;

	private ElapsedTime timer;

	private ElapsedTime moveForwardTimer;
	private ElapsedTime moveLeftTimer;
	private ElapsedTime moveRightTimer;

	private void moveForward() {
		double axial = 0.8, lateral = 0.0, yaw = 0.0;
		double leftFrontPower = axial + lateral + yaw;
		double rightFrontPower = axial - lateral - yaw;
		double leftBackPower = axial - lateral + yaw;
		double rightBackPower = axial + lateral - yaw;

		if (moveForwardTimer.time() < 0.34) {
			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);
		} else {
			leftFrontDrive.setPower(0);
			rightFrontDrive.setPower(0);
			leftBackDrive.setPower(0);
			rightBackDrive.setPower(0);
		}
	}

	private void moveRight() {
		double axial = 0.0, lateral = 0.8, yaw = 0.0;
		double leftFrontPower = axial + lateral + yaw;
		double rightFrontPower = axial - lateral - yaw;
		double leftBackPower = axial - lateral + yaw;
		double rightBackPower = axial + lateral - yaw;

		if (moveForwardTimer.time() < 0.46) {
			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);
		} else {
			leftFrontDrive.setPower(0);
			rightFrontDrive.setPower(0);
			leftBackDrive.setPower(0);
			rightBackDrive.setPower(0);
		}
	}

	private void moveLeft() {
		double axial = 0.0, lateral = -0.8, yaw = 0.0;
		double leftFrontPower = axial + lateral + yaw;
		double rightFrontPower = axial - lateral - yaw;
		double leftBackPower = axial - lateral + yaw;
		double rightBackPower = axial + lateral - yaw;

		if (moveForwardTimer.time() < 0.46) {
			leftFrontDrive.setPower(leftFrontPower);
			rightFrontDrive.setPower(rightFrontPower);
			leftBackDrive.setPower(leftBackPower);
			rightBackDrive.setPower(rightBackPower);
		} else {
			leftFrontDrive.setPower(0);
			rightFrontDrive.setPower(0);
			leftBackDrive.setPower(0);
			rightBackDrive.setPower(0);
		}
	}

	@Override
	public void runOpMode() {
		moveForwardTimer = new ElapsedTime();
		moveLeftTimer = new ElapsedTime();
		moveRightTimer = new ElapsedTime();
		timer = new ElapsedTime();
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(telemetry, Camera.FX, Camera.FY, Camera.CX, Camera.CY);
		camera = new Camera(telemetry, hardwareMap, "webcam1", aprilTagDetectionPipeline);

		leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
		rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
		extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");

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

		int numFramesWithoutDetection = 0;
		final float DECIMATION_HIGH = 3;
		final float DECIMATION_LOW = 2;
		final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
		final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

		final double FEET_PER_METER = 3.28084;
		timer.reset();
		Location location = null;

		ElapsedTime anotherTimer = new ElapsedTime();
		anotherTimer.reset();

		while (opModeIsActive()) {
			if (location == null) {
				Point point = aprilTagDetectionPipeline.updateInLoop();
				if (point != null) {
					if (rect1.contains(point)) {
						location = Location.LEFT;
						moveForwardTimer.reset();
						moveLeftTimer.reset();
						moveRightTimer.reset();
						anotherTimer.reset();
					} else if (rect2.contains(point)) {
						location = Location.CENTER;
						moveForwardTimer.reset();
						moveLeftTimer.reset();
						moveRightTimer.reset();
						anotherTimer.reset();
					} else if (rect3.contains(point)) {
						location = Location.RIGHT;
						moveForwardTimer.reset();
						moveLeftTimer.reset();
						moveRightTimer.reset();
						anotherTimer.reset();
					}
				}
			} else {
				telemetry.addLine(location.toString());
				telemetry.update();
				moveForward();
				if (location == Location.LEFT) {
					moveLeft();
				} else if (location == Location.RIGHT) {
					moveRight();
				}
				ArmSystem.moveServoToTarget(armServo, ArmSystem.ArmPositionSetting.LEFT_EXTENSION.arm, 0.004);
				ArmSystem.moveServoToTarget(elbowServo, ArmSystem.ArmPositionSetting.LEFT_EXTENSION.elbow, 0.004);
				if (anotherTimer.time() < 3.5) {
					extensionMotor.setPower(-0.8);
				} else {
					extensionMotor.setPower(0.0);
				}
			}

			// move left
//			double lateral = -0.6;
//			double axial = 0.0, yaw = 0.0;
//
//			double leftFrontPower = axial + lateral + yaw;
//			double rightFrontPower = axial - lateral - yaw;
//			double leftBackPower = axial - lateral + yaw;
//			double rightBackPower = axial + lateral - yaw;
//
//			if (timer.time() < 2.6) {
//				leftFrontDrive.setPower(leftFrontPower);
//				rightFrontDrive.setPower(rightFrontPower);
//				leftBackDrive.setPower(leftBackPower);
//				rightBackDrive.setPower(rightBackPower);
//			} else {
//				leftFrontDrive.setPower(0);
//				rightFrontDrive.setPower(0);
//				leftBackDrive.setPower(0);
//				rightBackDrive.setPower(0);
//			}
//			telemetry.update();
//			sleep(20);
		}
	}
}