package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SubSystem;
import org.firstinspires.ftc.teamcode.util.MotorUtil;

public class ArmSubSystem extends SubSystem {

	private static class ArmLevelSetting {
		// heighest
		public static ArmLevelSetting LEVEL3 = new ArmLevelSetting(0.53, 0.28);
		// middle
		public static ArmLevelSetting LEVEL2 = new ArmLevelSetting(0.316, 0.375);
		// lowest
		public static ArmLevelSetting LEVEL1 = new ArmLevelSetting(0.141, 0.454);
		public static ArmLevelSetting GROUND = new ArmLevelSetting(0.12765, 0.57089);

		public static ArmLevelSetting CONE_5 = new ArmLevelSetting(0.3112, 0.4731);
		public static ArmLevelSetting CONE_4 = new ArmLevelSetting(0.3379, 0.4904);
		public static ArmLevelSetting CONE_3 = new ArmLevelSetting(0.3098, 0.5045);
		public static ArmLevelSetting CONE_2 = new ArmLevelSetting(0.2820, 0.5184);

		private double arm;
		private double wrist;

		private ArmLevelSetting(double wrist, double arm) {
			this.wrist = wrist;
			this.arm = arm;
		}
	}

	// servo and motors
	private Servo armServo = null;
	private Servo wristServo = null;
	private Servo clawServo = null;

	private DcMotor shaftMotor = null;

	// arm
	private static final double ARM_MIN_POS = 0.0328;
	private static final double ARM_MAX_POS = 0.6;
	// pre-determined arm position constants
	private double armCounter = ARM_MIN_POS;

	// wrist
	private double wristCounter = WRIST_MIN;
	private static final double WRIST_MAX = 0.6468;
	private static final double WRIST_MIN = 0.12765;

	// claw
	private ElapsedTime clawTimer;
	private static final double CLAW_TOGGLE_COOLDOWN = 0.5;  // 0.5 sec

	// slow mode
	private ElapsedTime slowModeTimer;
	private static final double SLOW_TOGGLE_COOLDOWN = 0.3;
	private boolean slowMode = true;

	@Override
	public void init() {
		// set the timer
		clawTimer = new ElapsedTime();
		clawTimer.reset();
		slowModeTimer = new ElapsedTime();
		slowModeTimer.reset();

		// create servos
		clawServo = hardwareMap.get(Servo.class, "Servo1");
		armServo = hardwareMap.get(Servo.class, "Servo0");
		wristServo = hardwareMap.get(Servo.class, "Servo2");

		// create motors
		shaftMotor = hardwareMap.get(DcMotor.class, "shaftMotor");
	}

	@Override
	public void update() {
		// toggle slow mode
		if (Math.abs(gamepad1.left_trigger) > 0.05) {
			if (slowModeTimer.seconds() > SLOW_TOGGLE_COOLDOWN) {
				slowMode = !slowMode;
				slowModeTimer.reset();
			}
		}
		final double speedFactor = slowMode ? 1.0 : 2.0;

		// arm servo
		// set the position when stacking the cones
		// position 1: the lowest
		if (gamepad1.dpad_down) {
			armCounter = ArmLevelSetting.LEVEL1.arm;
			wristCounter = ArmLevelSetting.LEVEL1.wrist;
		}  // position 2
		else if (gamepad1.dpad_right) {
			armCounter = ArmLevelSetting.LEVEL2.arm;
			wristCounter = ArmLevelSetting.LEVEL2.wrist;
		} // position 3
		else if (gamepad1.dpad_up) {
			armCounter = ArmLevelSetting.LEVEL3.arm;
			wristCounter = ArmLevelSetting.LEVEL3.wrist;
		}
		// set the position when grabbing the cones
		else if (gamepad1.dpad_left) {
			armCounter = ArmLevelSetting.GROUND.arm;
			wristCounter = ArmLevelSetting.GROUND.wrist;
		} // second lowest
		else if (gamepad1.x) {
			armCounter = ArmLevelSetting.CONE_2.arm;
			wristCounter = ArmLevelSetting.CONE_2.wrist;
		} // third lowest
		else if (gamepad1.y) {
			armCounter = ArmLevelSetting.CONE_3.arm;
			wristCounter = ArmLevelSetting.CONE_3.wrist;
		}  // fourth cone
		else if (gamepad1.b) {
			armCounter = ArmLevelSetting.CONE_4.arm;
			wristCounter = ArmLevelSetting.CONE_4.wrist;
		} // fifth/top cone
		else if (gamepad1.a) {
			armCounter = ArmLevelSetting.CONE_5.arm;
			wristCounter = ArmLevelSetting.CONE_5.wrist;
		}
		// if none of the buttons are pressed
		else {
			final double armSpeed = 0.0005 * speedFactor;
			armCounter += armSpeed * gamepad1.right_stick_y;
			wristCounter -= armSpeed * gamepad1.right_stick_y * 2.0;
			wristCounter = MotorUtil.clamp(wristCounter, WRIST_MIN, WRIST_MAX);
			armCounter = MotorUtil.clamp(armCounter, ARM_MIN_POS, ARM_MAX_POS);
		}
		armServo.setPosition(armCounter);
		wristServo.setPosition(wristCounter);

		// wrist servo
		if (gamepad1.left_stick_y != 0.0) {
			final double wristSpeed = 0.001 * speedFactor;
			wristCounter += wristSpeed * gamepad1.left_stick_y;
			// clamp the wrist position
			wristCounter = MotorUtil.clamp(wristCounter, WRIST_MIN, WRIST_MAX);
		}
		wristServo.setPosition(wristCounter);

		// claw servo
		// if trigger is toggled
		if (Math.abs(gamepad1.right_trigger) > 0.05f) {
			if (clawTimer.seconds() > CLAW_TOGGLE_COOLDOWN) {
				if (clawServo.getPosition() < 35.0 / 300.0) {
					clawServo.setPosition(70.0 / 300.0);
				} else {
					clawServo.setPosition(0.0 / 300.0);
				}
				clawTimer.reset();
			}
		}

		// shaft motor
		final double shaftSpeed = 0.5 * speedFactor;
		if (gamepad1.left_bumper) {
			// go down
			shaftMotor.setPower(shaftSpeed);
		} else if (gamepad1.right_bumper) {
			// go up
			shaftMotor.setPower(-shaftSpeed);
		} else {
			shaftMotor.setPower(0.0);
		}

		// debug
		telemetry.addLine("Wrist Pos: " + wristServo.getPosition());
		telemetry.addLine("Arm Pos: " + armServo.getPosition());
		telemetry.update();
	}
}