package cg;

import java.awt.geom.Point2D;
import java.util.ArrayList;

class EnemyWave {
    Point2D.Double fireLocation;

    long fireTime;
    double bulletVelocity, directAngle, distanceTraveled, bulletPower, playerDistance, maxEscapeAngle;
    int direction, weight;

    ArrayList safePoints;
    double[] waveGuessFactors;

    public EnemyWave() {
    }

    public double distanceToPoint(Point2D.Double p) {
        return fireLocation.distance(p);
    }

    public double absoluteBearing(Point2D.Double target) {
        return Math.atan2(target.x - fireLocation.x, target.y - fireLocation.y);
    }

}
