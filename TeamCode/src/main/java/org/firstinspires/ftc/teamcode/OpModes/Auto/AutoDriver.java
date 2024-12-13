package org.firstinspires.ftc.teamcode.OpModes.Auto;

import org.firstinspires.ftc.teamcode.Utilities.AutoActions;
import org.firstinspires.ftc.teamcode.Utilities.Action;

import java.util.ArrayList;

// TODO: This isn't true anymore. Teams must now make their own tasks like DriveTo or ArmUp.
/*
	This is a driver for 23684's robot. To use this you must create your own driver.
	This is similar to a roadrunner Localizer.
	If anyone on team 23684 is reading this, just update this file to the current robot.
	This Driver was made on December 8, 2024 for our two dead wheel, GoBilda Pinpoint Driver,
	and four mecanum wheel drive train.
	TODO For 23684 Devs: Make a Interface called driver that this should implement
 */
public class AutoDriver {
	public AutoActions autoActions;

	public AutoDriver(AutoActions autoActions){
		this.autoActions = autoActions;
	}

	public void run(){
		// forEach isn't being used due to performance. It increases time by 6µ (six microseconds) per item compared to a for loop and a base 24-36 bytes instead of 20-34 bytes that a for loop takes.
		ArrayList<Action> actions = autoActions.getActions();
		ArrayList<String> ids = autoActions.getIds();
		ArrayList<Integer> idsToRemove = new ArrayList<Integer>();

		for (int i = 0; i < actions.size(); i++) {
			Action action = actions.get(i);
			if (!ids.contains(action.getId())){
				if (action.getTask().run()){
					idsToRemove.add(i);
				}
			}
		}

		for (int i = 0; i < idsToRemove.size(); i++) {
			Integer id = idsToRemove.get(i);
			actions.remove(i);
			ids.remove(i);
		}
	}
}
