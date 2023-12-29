package org.firstinspires.ftc.teamcode.util;

public class MotorUtil {
	public static double clamp(double a, double min, double max) {
		if (a < min) return min;
		if (a > max) return max;
		return a;
	}

}
