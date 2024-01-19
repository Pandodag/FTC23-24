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
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.opencv.core.Rect;

@Autonomous
public class FinalAutonomousClose extends LinearOpMode {

	private Camera camera;
	private AprilTagDetectionPipeline aprilTagDetectionPipeline;

	// motors
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;

	private Rect rect1 = new Rect(0, 0, Camera.CAMERA_WIDTH / 3, Camera.CAMERA_HEIGHT);
	private Rect rect2 = new Rect(rect1.x, 0, Camera.CAMERA_WIDTH / 3, Camera.CAMERA_HEIGHT);
	private Rect rect3 = new Rect(rect2.x, 0, Camera.CAMERA_WIDTH / 3, Camera.CAMERA_HEIGHT);

	private enum Location {
		LEFT, CENTER, RIGHT;
	}

	private Servo armServo = null;
	private Servo elbowServo = null;
	private Servo wristServo = null;
	private Servo handServo = null;

	private ElapsedTime timer;

	@Override
	public void runOpMode() {
		timer = new ElapsedTime();
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(telemetry, Camera.FX, Camera.FY, Camera.CX, Camera.CY);
		camera = new Camera(telemetry, hardwareMap, "webcam1", aprilTagDetectionPipeline);

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

		while (opModeIsActive()) {
			// move left
			double lateral = -0.6;
			double axial = 0.0, yaw = 0.0;

			double leftFrontPower = axial + lateral + yaw;
			double rightFrontPower = axial - lateral - yaw;
			double leftBackPower = axial - lateral + yaw;
			double rightBackPower = axial + lateral - yaw;

			if (timer.time() < 1.6) {
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
			telemetry.update();
//			sleep(20);
		}
	}
}