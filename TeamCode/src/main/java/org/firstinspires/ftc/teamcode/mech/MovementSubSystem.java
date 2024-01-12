package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SubSystem;

public class MovementSubSystem extends SubSystem {

	// Declare OpMode members for each of the 4 motors.
	private ElapsedTime runtime = new ElapsedTime();
	private DcMotor leftFrontDrive = null;
	private DcMotor leftBackDrive = null;
	private DcMotor rightFrontDrive = null;
	private DcMotor rightBackDrive = null;

	// slow mode
	private boolean slowMode = false;
	private ElapsedTime slowModeTimer = new ElapsedTime();
	private static final double SLOW_TOGGLE_COOLDOWN = 0.3;

	@Override
	public void init() {
		leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
		leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
		rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

		leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
		leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
		rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
		rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

		// Wait for the game to start (driver presses PLAY)
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		runtime.reset();
		slowModeTimer.reset();
	}

	@Override
	public void update() {
		// toggle slow mode
		if (gamepad2.x) {
			if (slowModeTimer.seconds() > SLOW_TOGGLE_COOLDOWN) {
				slowMode = !slowMode;
				slowModeTimer.reset();
			}
		}

		double speedFactor = slowMode ? 0.5 : 1.0;

		double max;

		// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
		double axial = -gamepad2.left_stick_y;  // Note: pushing stick forward gives negative value
		double lateral = gamepad2.left_stick_x;
		double yaw = gamepad2.right_stick_x;

		// Combine the joystick requests for each axis-motion to determine each wheel's power.
		// Set up a variable for each drive wheel to save the power level for telemetry.
		double leftFrontPower = (axial + lateral + yaw) * speedFactor;
		double rightFrontPower = (axial - lateral - yaw) * speedFactor;
		double leftBackPower = (axial - lateral + yaw) * speedFactor;
		double rightBackPower = (axial + lateral - yaw) * speedFactor;

		// Normalize the values so no wheel power exceeds 100%
		// This ensures that the robot maintains the desired motion.
		max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
		max = Math.max(max, Math.abs(leftBackPower));
		max = Math.max(max, Math.abs(rightBackPower));
		if (max > 1.0) {
			leftFrontPower /= max;
			rightFrontPower /= max;
			leftBackPower /= max;
			rightBackPower /= max;
		}

		// Send calculated power to wheels
		leftFrontDrive.setPower(leftFrontPower);
		rightFrontDrive.setPower(rightFrontPower);
		leftBackDrive.setPower(leftBackPower);
		rightBackDrive.setPower(rightBackPower);

		// Show the elapsed game time and wheel power.
		telemetry.addData("Status", "Run Time: " + runtime.toString());
		telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
		telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
		telemetry.update();
	}
}
