package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubSystem {
	protected static Telemetry telemetry;
	protected static HardwareMap hardwareMap;
	protected static Gamepad gamepad1;
	protected static Gamepad gamepad2;

	public static void initSubSystems(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
		SubSystem.telemetry = telemetry;
		SubSystem.hardwareMap = hardwareMap;
		SubSystem.gamepad1 = gamepad1;
		SubSystem.gamepad2 = gamepad2;
	}

	public SubSystem() {

	}

	public abstract void init();

	public abstract void update();
}
