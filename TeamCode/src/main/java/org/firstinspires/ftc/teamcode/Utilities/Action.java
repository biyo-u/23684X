package org.firstinspires.ftc.teamcode.Utilities;

public class Action {
	private final Task task;
	private final String id;
	private final Action requirement;

	public Action(Task task, String id, Action requirement){
		this.task = task;
		this.id = id;
		this.requirement = requirement;
	}

	public Action getRequirement(){
		return requirement;
	}

	public String getId(){
		return id;
	}

	public Task getTask(){
		return task;
	}
}
