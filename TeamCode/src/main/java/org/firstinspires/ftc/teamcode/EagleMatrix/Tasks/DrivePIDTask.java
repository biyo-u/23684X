package org.firstinspires.ftc.teamcode.EagleMatrix.Tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Task;

public class DrivePIDTask extends Task {

	double Kp = 0;
	double Ki = 0;
	double Kd = 0;

	// TODO: Find out what this is
	double reference = 0;
	double lastReference = 0;

	double integralSum = 0;

	double lastError = 0;

	// TODO: Find this value after tuning: maxIntegralSum * Ki = 0.25
	double maxIntegralSum = 0;

	double a = 0.8; // a can be anything from 0 < a < 1
	double previousFilterEstimate = 0;
	double currentFilterEstimate;

	@Override
	public boolean run() {
		ElapsedTime timer = new ElapsedTime();

		// TODO: Find this
		double encoderPosition = 0;
		// calculate the error
		double error = reference - encoderPosition;

		double errorChange = (error - lastError);

		// filter out height frequency noise to increase derivative performance
		currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
		previousFilterEstimate = currentFilterEstimate;

		// rate of change of the error
		double derivative = currentFilterEstimate / timer.seconds();

		// sum of all error over time
		integralSum = integralSum + (error * timer.seconds());


		// max out integral sum
		if (integralSum > maxIntegralSum) {
			integralSum = maxIntegralSum;
		}

		if (integralSum < -maxIntegralSum) {
			integralSum = -maxIntegralSum;
		}

		// reset integral sum upon setpoint changes
		if (reference != lastReference) {
			integralSum = 0;
		}

		double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

		// TODO: Set power to motors with power out

		lastError = error;

		lastReference = reference;

		// reset the timer for next time
		timer.reset();

		// TODO: Check for this
		boolean reachedTarget = false;

		return reachedTarget;
	}
}
