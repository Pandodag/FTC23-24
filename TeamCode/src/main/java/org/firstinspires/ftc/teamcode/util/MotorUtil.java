package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MotorUtil {
	public static double clamp(double a, double min, double max) {
		if (a < min) return min;
		if (a > max) return max;
		return a;
	}

	public static void moveServoToTarget(Servo s, double targetPosition, double inc) {
		if (s.getPosition() == targetPosition) {
			return;
		}
		double moveAmount = targetPosition - s.getPosition();
		double dir = Math.signum(moveAmount);
		moveAmount = Math.min(Math.abs(moveAmount), inc);
		s.setPosition(s.getPosition() + moveAmount * dir);
	}

	public static void moveBot(
			DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
			double axial, double lateral, double yaw) {
		double leftFrontPower = axial + lateral + yaw;
		double rightFrontPower = axial - lateral - yaw;
		double leftBackPower = axial - lateral + yaw;
		double rightBackPower = axial + lateral - yaw;

		// normalize by the maximum speed if it exceeds 100%
		double max;
		max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
		max = Math.max(max, Math.abs(leftBackPower));
		max = Math.max(max, Math.abs(rightBackPower));

		if (max > 1.0) {
			leftFrontPower /= max;
			rightFrontPower /= max;
			leftBackPower /= max;
			rightBackPower /= max;
		}

		lf.setPower(leftFrontPower);
		rf.setPower(rightFrontPower);
		lb.setPower(leftBackPower);
		rb.setPower(rightBackPower);
	}

	public static void strafeBot(
			DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
			double lateral) {
		moveBot(lf, lb, rf, rb, 0.0, lateral, 0.0);
	}

	public static void moveForward(
			DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
			double axial) {
		moveBot(lf, lb, rf, rb, axial, 0.0, 0.0);
	}
}
