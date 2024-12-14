package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;

public class AutoActions {
	ArrayList<Action> actions = new ArrayList<Action>();
	ArrayList<String> ids = new ArrayList<String>();
	ArrayList<String> requirements = new ArrayList<String>();

	public AutoActions(){

	}

	public AutoActions add(Action action){
		actions.add(action);
		ids.add(action.getId());
		requirements.add(action.getRequirement().getId());
		return this;
	}

	public ArrayList<Action> getActions(){
		return actions;
	}

	public ArrayList<String> getIds(){
		return ids;
	}
}
