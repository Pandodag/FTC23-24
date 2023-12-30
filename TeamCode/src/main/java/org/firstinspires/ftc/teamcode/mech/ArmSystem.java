package org.firstinspires.ftc.teamcode.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.SubSystem;

public class ArmSystem extends SubSystem {

	private static class ArmPositionSetting {
		public static final ArmPositionSetting LEFT_EXTENSION
				= new ArmPositionSetting(0.0, 0.2679, 0.8552);

		public static final ArmPositionSetting UPWARD_EXTENSION
				= new ArmPositionSetting(0.0, 0.07806, 0.8895);

		public static final ArmPositionSetting CLOSED_EXTENSION
				= new ArmPositionSetting(0.0, 0.2679, 0.3439);

		public static final ArmPositionSetting A_EXTENSION = new ArmPositionSetting(0.0, 0.0, 1.0);
		public static final ArmPositionSetting B_EXTENSION = new ArmPositionSetting(0.0, 0.036, 1.0);
		public static final ArmPositionSetting Y_EXTENSION = new ArmPositionSetting(0.0, 0.0679, 1.0);

		// arm
		public double extension;
		public double arm;
		public double elbow;

		public ArmPositionSetting(double extension, double arm, double elbow) {
			this.extension = extension;
			this.arm = arm;
			this.elbow = elbow;
		}
	}

	private static class WristPositionSetting {
		// hand will be open
		public static final WristPositionSetting PLACING_POSITION = new WristPositionSetting(0.716,true);
		// hand will be closed
		public static final WristPositionSetting GRABBING_POSITION = new WristPositionSetting(0.0499, false);

		// claw
		public double wrist;
		public double hand;

		public boolean open;

		public WristPositionSetting(double wrist, boolean open) {
			this.wrist = wrist;
			this.open = open;
			setHand();
		}

		public void setHand() {
			if (open) {
				hand = 0.00655;
			} else {
				hand = 0.2296;
			}
		}
	}

	private DcMotor extensionMotor = null;
	private Servo armServo = null;
	private Servo elbowServo = null;
	private Servo wristServo = null;
	private Servo handServo = null;

	private ArmPositionSetting currentArmPosition = ArmPositionSetting.CLOSED_EXTENSION;
	private WristPositionSetting currentWristPosition = WristPositionSetting.GRABBING_POSITION;

	// claw
	private ElapsedTime clawTimer;
	private static final double CLAW_TOGGLE_COOLDOWN = 0.5;  // 0.5 sec

	private void setData(ArmPositionSetting a, WristPositionSetting w) {
		extensionMotor.setTargetPosition((int) a.extension);
		armServo.setPosition(a.arm);
		elbowServo.setPosition(a.elbow);
		wristServo.setPosition(w.wrist);
		handServo.setPosition(w.hand);
	}

	@Override
	public void init() {
		clawTimer = new ElapsedTime();
		clawTimer.reset();
		// find servos on hardware map
		extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
		armServo = hardwareMap.get(Servo.class, "armServo");
		elbowServo = hardwareMap.get(Servo.class, "elbowServo");
		wristServo = hardwareMap.get(Servo.class, "wristServo");
		handServo = hardwareMap.get(Servo.class, "handServo");
	}

	@Override
	public void update() {
		// big arm code
		// left extension
		if (gamepad1.dpad_left) {
			currentArmPosition = ArmPositionSetting.LEFT_EXTENSION;
			currentWristPosition = WristPositionSetting.GRABBING_POSITION;
			currentWristPosition.open = true;
			currentWristPosition.setHand();
		}
		// upward position
		else if (gamepad1.y) {
			currentArmPosition = ArmPositionSetting.Y_EXTENSION;
			currentWristPosition = WristPositionSetting.PLACING_POSITION;
			currentWristPosition.open = false;
			currentWristPosition.setHand();
		}
		// close position
		else if (gamepad1.dpad_down) {
			currentArmPosition = ArmPositionSetting.CLOSED_EXTENSION;
		}

		// claw code
		// original position
		if (gamepad1.a) {
			currentArmPosition = ArmPositionSetting.A_EXTENSION;
			currentWristPosition = WristPositionSetting.PLACING_POSITION;
			currentWristPosition.open = false;
			currentWristPosition.setHand();
		}
		// twist wrist servo and flip the hand servo up
		else if (gamepad1.b) {
			currentArmPosition = ArmPositionSetting.B_EXTENSION;
			currentWristPosition = WristPositionSetting.PLACING_POSITION;
			currentWristPosition.open = false;
			currentWristPosition.setHand();
		}
		if (gamepad1.x) {
			if (clawTimer.seconds() > CLAW_TOGGLE_COOLDOWN) {
				if (currentWristPosition.open) { // if open, close
					currentWristPosition.hand = WristPositionSetting.PLACING_POSITION.hand;
					currentWristPosition.open = false;
					currentWristPosition.setHand();
				} else { // open
					currentWristPosition.hand = WristPositionSetting.GRABBING_POSITION.hand;
					currentWristPosition.open = true;
					currentWristPosition.setHand();
				}
				clawTimer.reset();
			}
		}
		setData(currentArmPosition, currentWristPosition);
		currentArmPosition.arm += gamepad1.right_stick_y * 0.001f;

		extensionMotor.setPower(gamepad1.left_stick_y);

		telemetry.addLine("Extension Motor: " + extensionMotor.getCurrentPosition() + " " + extensionMotor.getTargetPosition());
		telemetry.addLine("arm " + armServo.getPosition() + " " + currentArmPosition.arm);
		telemetry.addLine("elbow " + elbowServo.getPosition() + " " + currentArmPosition.elbow);
		telemetry.addLine("wrist " + wristServo.getPosition());
		telemetry.addLine("hand " + handServo.getPosition());
		telemetry.update();
	}
}
