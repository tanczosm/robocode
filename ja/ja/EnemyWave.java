package ja;

import robocode.*;
import robocode.util.Utils;
import java.awt.*;
import java.awt.geom.*;     // for Point2D's
import java.util.ArrayList; // for collection of waves

class EnemyWave {
        Point2D.Double fireLocation;

        long fireTime;
        double bulletVelocity, directAngle, distanceTraveled, bulletPower, playerDistance, maxEscapeAngle;
        int direction, weight;
		
		double[] waveGuessFactors;
 
        public EnemyWave() { }
		
        public double distanceToPoint(Point2D.Double p) {
            return fireLocation.distance(p);
        }
        public double absoluteBearing(Point2D.Double target) {
            return Math.atan2(target.x - fireLocation.x, target.y - fireLocation.y);
        }
 
}
