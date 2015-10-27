package cg;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Comparator;

class EnemyWave implements Comparable<EnemyWave> {
    Point2D.Double fireLocation;

    long fireTime;
    double bulletVelocity, directAngle, distanceTraveled, bulletPower, playerDistance, maxEscapeAngle;
    int direction, weight;

    // Temp data only for sorting - DO NOT USE
    public double currentDistanceToPlayer;

    // Additional data
    public double lateralVelocity;
    public double advancingVelocity;
    public double lateralDistanceLast10;
    public double acceleration;
    public int timeSinceDirectionChange;
    public double forwardWallDistance;
    public double reverseWallDistance;


    // Used by the pathing predictor, where we will end at when we are at the last point on safePoints
    Point2D.Double predictedPosition;
    double predictedVelocity;
    double predictedHeading;

    ArrayList safePoints;
    double[] waveGuessFactors;

    public boolean redirected = false;

    public EnemyWave() {
    }

    public double distanceToPoint(Point2D.Double p) {
        return fireLocation.distance(p);
    }

    public double absoluteBearing(Point2D.Double target) {
        return Math.atan2(target.x - fireLocation.x, target.y - fireLocation.y);
    }

    public int compareTo(EnemyWave other)
    {
        if (currentDistanceToPlayer < other.currentDistanceToPlayer)
            return -1;
        else if (currentDistanceToPlayer == other.currentDistanceToPlayer)
            return 0;

        return 1;
    }
}
