package ja;

import java.awt.geom.Point2D;

public class RobotState {
	public Point2D.Double location;
	public double heading;
	public double velocity;
	public long time;
	public boolean smoothing;
	public double danger;
	public int tickDistance;
	public double dangerDensity; // When combined with adjacent states, how dangerous is this spot?
	
	public RobotState(Point2D.Double botLocation, double botHeadingRadians,
		double botVelocity) {

		location = botLocation;
		heading = botHeadingRadians;
		velocity = botVelocity;
		smoothing = false;
	}
	
	public RobotState(Point2D.Double botLocation, double botHeadingRadians,
		double botVelocity, long currentTime) {
		
		this(botLocation, botHeadingRadians, botVelocity);
		
		time = currentTime;
	}

	public RobotState(Point2D.Double botLocation, double botHeadingRadians,
			double botVelocity, long currentTime, boolean smooth) {
		
		this(botLocation, botHeadingRadians, botVelocity, currentTime);
		
		smoothing = smooth;
	}

	public Object clone() {
		return new RobotState((Point2D.Double)location.clone(), heading, 
			velocity, time, smoothing);
	}

}
