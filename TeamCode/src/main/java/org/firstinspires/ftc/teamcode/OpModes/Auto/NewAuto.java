package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Action;
import org.firstinspires.ftc.teamcode.Utilities.AutoActions;

@TeleOp(name="Auto", group= Constants.GroupNames.Autonomous)
public class NewAuto extends OpMode {
	AutoActions autoActions;

	@Override
	public void init() {
//		autoActions = new AutoActions().add(new Action());
	}

	@Override
	public void loop() {

	}
}