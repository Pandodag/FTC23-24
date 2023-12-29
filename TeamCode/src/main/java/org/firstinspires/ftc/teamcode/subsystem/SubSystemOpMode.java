package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class SubSystemOpMode extends OpMode {
	protected ArrayList<SubSystem> subSystems = new ArrayList<>();

	public SubSystemOpMode() {
	}

	public void addSubSystem(SubSystem subSystem) {
		subSystems.add(subSystem);
	}

	@Override
	public void init() {
		SubSystem.initSubSystems(telemetry, hardwareMap, gamepad1, gamepad2);
		for (SubSystem subSystem : subSystems) {
			subSystem.init();
		}
	}

	@Override
	public void loop() {
		for (SubSystem subSystem : subSystems) {
			subSystem.update();
		}
	}
}
