package cg;

import robocode.Bullet;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;

class EnemyWave implements Comparable<EnemyWave> {

    private static final double MAX_ESCAPE_FACTOR = 1.1;

    Point2D.Double fireLocation;
    public Rectangle2D.Double hitbox = new Rectangle2D.Double();

    long fireTime;
    double bulletVelocity, directAngle, distanceTraveled, bulletPower, playerDistance, maxEscapeAngle;
    int direction, weight;
    double dweight;

    boolean collidedWithBullet = false;

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


    public Point2D.Double lastTarget = new Point2D.Double();
    public double minFactor = java.lang.Double.POSITIVE_INFINITY;
    public double maxFactor = java.lang.Double.NEGATIVE_INFINITY;

    // Used by the pathing predictor, where we will end at when we are at the last point on safePoints
    /*
    Point2D.Double predictedPosition;
    double predictedVelocity;
    double predictedHeading;
    */

    ArrayList<RobotState> safePoints;
    double[] waveGuessFactors = null;
    double[] waveGuessFactorsRand;

    public int escapeDirection = 1; // What direction is player traveling relative to fireLocation to move to safest point
    public boolean imaginary = false;
    public boolean redirected = false;

    public EnemyWave() {
    }

    public double distanceToPoint(Point2D.Double p) {
        return fireLocation.distance(p);
    }

    public double absoluteBearing(Point2D.Double target) {
        return Math.atan2(target.x - fireLocation.x, target.y - fireLocation.y);
    }

    public double absoluteBearingFrom(Point2D.Double target) {
        return Math.atan2(fireLocation.x - target.x, fireLocation.y - target.y);
    }

    public double getGuessFactor(Point2D.Double targetLocation) {
        double offsetAngle = (CTUtils.absoluteBearing(fireLocation, targetLocation)
                - directAngle);
        double factor = Utils.normalRelativeAngle(offsetAngle)
                / CTUtils.maxEscapeAngle(bulletVelocity) * direction;

        return factor;
    }

    public int compareTo(EnemyWave other)
    {
        if (currentDistanceToPlayer < other.currentDistanceToPlayer)
            return -1;
        else if (currentDistanceToPlayer == other.currentDistanceToPlayer)
            return 0;

        return 1;
    }


    public int intersected = 0;

    public double getRadius(long time) {
        return bulletVelocity*(time - fireTime);
    }

    /**
     * Used to determine if we can safely remove this wave
     */
    public final boolean didIntersect(Point2D.Double target, long time) {

        hitbox.setFrame(target.x, target.y, 36, 36);

        double radius = getRadius(time);
        double nextRadius = getRadius(time+1);
        Point2D.Double[] current = CTUtils.intersectRectCircle(hitbox, fireLocation, radius);
        Point2D.Double[] next = CTUtils.intersectRectCircle(hitbox, fireLocation, nextRadius);

        if(current.length != 0 || next.length != 0) {
            ++intersected;
        } else {
            if(intersected > 0) {
                return true;
            }
        }

        return false;
    }

    // Comes from nene robot, under ZLIB license
    public final boolean intersects(Point2D.Double target, long time) {
        hitbox.setFrame(target.x, target.y, 36, 36);


        double radius = getRadius(time);
        double nextRadius = getRadius(time+1);
        Point2D.Double[] current = CTUtils.intersectRectCircle(hitbox, fireLocation, radius);
        Point2D.Double[] next = CTUtils.intersectRectCircle(hitbox, fireLocation, nextRadius);

        if(current.length != 0 || next.length != 0) {
            return true;
        }

        return false;
    }

    /**
     * Used for calculating where a bullet will intersect a target
     */
    public final boolean doesIntersect(Point2D.Double target, long time) {
        hitbox.setFrame(target.x, target.y, 36, 36);

        double radius = getRadius(time);
        double nextRadius = getRadius(time+1);
        double[][] current = CTUtils.intersectRectCircleD(hitbox, fireLocation, radius);
        double[][] next = CTUtils.intersectRectCircleD(hitbox, fireLocation, nextRadius);

        if(current.length != 0 || next.length != 0) {
            for(double[] v : current)
                expandFactors(v);

            for(double[] v : next)
                expandFactors(v);

            double[][] corners = new double[4][2]; //hitbox.getCorners();
            for(double[] c : corners) {
                double dist = fireLocation.distanceSq(new Point2D.Double(c[0], c[1]));
                if(dist < nextRadius*nextRadius
                        && dist > radius*radius) {
                    expandFactors(c);
                }
            }

            return true;
        }

        return false;
    }

    public final double[][] getCorners(Rectangle2D.Double r) {
        return new double[][] {
                {r.x,r.y},
                {r.x,r.y+r.height},
                {r.x+r.width,r.y},
                {r.x+r.width,r.y+r.height},
        };
    }

    public final double[][] getEdges(Rectangle2D.Double r) {
        return new double[][] {
                {r.x,r.y,r.x+r.width,r.y},
                {r.x,r.y,r.x,r.y+r.height},
                {r.x,r.y+r.height,r.x+r.width,r.y+r.height},
                {r.x+r.width,r.y,r.x+r.width,r.y+r.height}
        };
    }    

    public ArrayList<double[]> bulletShadows = new ArrayList<double[]>();
    public ArrayList<BulletShadow> unmergedShadows = new ArrayList<BulletShadow>();

    public final void shadowBullet(Bullet b, Line line, long time, Graphics g) {

        if (!b.isActive())
            return;

        double minFactor = java.lang.Double.POSITIVE_INFINITY;
        double maxFactor = java.lang.Double.NEGATIVE_INFINITY;

        boolean intersect = false;

        double radius = getRadius(time);
        double nextRadius = getRadius(time+1);
        //System.out.println("radius: " + radius + ", nextRadius: " + nextRadius + ", line: [" + line.x1 + "," + line.y1 + "," + line.x2 + "," + line.y2 + "]");

        double[] current = CTUtils.intersectSegCircle(fireLocation.x, fireLocation.y, radius, line.x1, line.y1, line.x2, line.y2);
        double[] next = CTUtils.intersectSegCircle(fireLocation.x, fireLocation.y, nextRadius, line.x1, line.y1, line.x2, line.y2);

        if (next.length > 0 || current.length > 0) {
            g.drawOval((int)(fireLocation.x-(radius)), (int)(fireLocation.y-(radius)), (int)radius*2, (int)radius*2);
            g.drawOval((int)(fireLocation.x-(nextRadius)), (int)(fireLocation.y-(nextRadius)), (int)nextRadius*2, (int)nextRadius*2);
        }

        for(int i=0; i<current.length; i+=2) {
            //double angle = Utils.normalRelativeAngle(absoluteBearing(new Point2D.Double(current[i], current[i + 1])) - directAngle) / maxEscapeAngle;
            double angle = getGuessFactor(new Point2D.Double(current[i], current[i + 1]));

            if(angle < minFactor) minFactor = angle;
            if(angle > maxFactor) maxFactor = angle;

            intersect = true;
        }

        for(int i=0; i<next.length; i+=2) {
            Point2D.Double pt = new Point2D.Double(next[i], next[i + 1]);
            g.setColor(Color.CYAN);
            g.drawOval((int)pt.x-5, (int)pt.y-5, 10, 10);
            g.drawLine((int)line.x1, (int)line.y1, (int)line.x2, (int)line.y2);

            //double angle = Utils.normalRelativeAngle(absoluteBearing(new Point2D.Double(next[i], next[i + 1])) - directAngle) / maxEscapeAngle;
            double angle = getGuessFactor(new Point2D.Double(next[i], next[i + 1]));
            //System.out.println("angle: " + angle + ", bearing: " + absoluteBearing(new Point2D.Double(next[i], next[i + 1])));
            if(angle < minFactor) minFactor = angle;
            if(angle > maxFactor) maxFactor = angle;
            intersect = true;
        }

        //if()
        double distA = fireLocation.distanceSq(line.x1, line.y1);
        if(distA < nextRadius*nextRadius && distA > radius*radius) {
            //double angle = Utils.normalRelativeAngle(absoluteBearing(new Point2D.Double(line.x1,line.y1)) - directAngle) / maxEscapeAngle;
            double angle = getGuessFactor(new Point2D.Double(line.x1, line.y1));

            if(angle < minFactor) minFactor = angle;
            if(angle > maxFactor) maxFactor = angle;
            intersect = true;
        }

        double distB = fireLocation.distanceSq(line.x2, line.y2);
        if(distB < nextRadius*nextRadius && distB > radius*radius) {
            //double angle = Utils.normalRelativeAngle(absoluteBearing(new Point2D.Double(line.x2, line.y2)) - directAngle) / maxEscapeAngle;
            double angle = getGuessFactor(new Point2D.Double(line.x2, line.y2));

            if(angle < minFactor) minFactor = angle;
            if(angle > maxFactor) maxFactor = angle;
            intersect = true;
        }

        if(intersect) {

            BulletShadow shadow = new BulletShadow();
            shadow.b = b;
            shadow.shadow = new double[] {
                    minFactor, maxFactor
            };

            //System.out.println("minFactor: " + minFactor + ", maxFactor: " + maxFactor);

            //if shadow is outside of the escape angles, don't add it
            if((minFactor > MAX_ESCAPE_FACTOR && maxFactor > MAX_ESCAPE_FACTOR)
                    || (minFactor < -MAX_ESCAPE_FACTOR && maxFactor < -MAX_ESCAPE_FACTOR)) {
                return;
            }

            //if one of the factors is outside of the escape angle, clamp it
            minFactor = CTUtils.limit(-MAX_ESCAPE_FACTOR, minFactor, MAX_ESCAPE_FACTOR);
            maxFactor = CTUtils.limit(-MAX_ESCAPE_FACTOR, maxFactor, MAX_ESCAPE_FACTOR);



            unmergedShadows.add(shadow);
            mergeShadow(shadow);
        }
    }

    public final void mergeShadow(BulletShadow shadow) {
        double minFactor = shadow.shadow[0];
        double maxFactor = shadow.shadow[1];

        boolean merged = false;
        for(double[] d : bulletShadows) {
            if(!(minFactor > d[1] || maxFactor < d[0])) {
                //intersection
                if(minFactor < d[0] && maxFactor > d[1]) {
                    d[0] = minFactor;
                    d[1] = maxFactor;
                }

                if(maxFactor > d[0] && maxFactor < d[1]) {
                    if(minFactor < d[0]) {
                        d[0] = minFactor;
                    }
                }
                if(minFactor < d[1] && minFactor > d[0]) {
                    if(maxFactor > d[1]) {
                        d[1] = maxFactor;
                    }
                }
                merged = true;
                break;
            }
        }

        if(!merged) {
            bulletShadows.add(shadow.shadow);
        }
    }

    public final void removeShadow(Bullet b) {
        boolean removed = false;
        Iterator<BulletShadow> it = unmergedShadows.iterator();
        while(it.hasNext()) {
            BulletShadow bs = it.next();
            if(bs.equals(b)) {
                it.remove();
                removed = true;
            }
            //we may have more then one shadow for each bullet
            //weird I know
        }

        if(removed) {
            bulletShadows.clear();

            //remerge all still existing shadows
            for(BulletShadow bs : unmergedShadows) {
                mergeShadow(bs);
            }
        }
    }

    public final void standingIntersection(Point2D.Double target) {
		/*
		 * hitbox.set(target, 36, 36);
		 * Point2D.Double[] corners = hitbox.getFasterCorners();
		 * for(Point2D.Double v : corners) {
		 *     expandFactors(v);
		 * }
		 *
		 * Equivalent, the below is just faster and uses less memory
		 */
        double angle = Utils.normalRelativeAngle(Math.atan2((target.x - 18) - fireLocation.x, (target.y - 18) - fireLocation.y)
                - directAngle) / maxEscapeAngle;
        if(angle < minFactor) minFactor = angle;
        if(angle > maxFactor) maxFactor = angle;

        angle = Utils.normalRelativeAngle(Math.atan2((target.x + 18) - fireLocation.x, (target.y - 18) - fireLocation.y)
                - directAngle) / maxEscapeAngle;
        if(angle < minFactor) minFactor = angle;
        if(angle > maxFactor) maxFactor = angle;

        angle = Utils.normalRelativeAngle(Math.atan2((target.x - 18) - fireLocation.x, (target.y + 18) - fireLocation.y)
                - directAngle) / maxEscapeAngle;
        if(angle < minFactor) minFactor = angle;
        if(angle > maxFactor) maxFactor = angle;

        angle = Utils.normalRelativeAngle(Math.atan2((target.x + 18) - fireLocation.x, (target.y + 18) - fireLocation.y)
                - directAngle) / maxEscapeAngle;
        if(angle < minFactor) minFactor = angle;
        if(angle > maxFactor) maxFactor = angle;

        lastTarget = (Point2D.Double)target.clone();
    }


    /**
     * Expand the guessfactors based on target location
     */
    private void expandFactors(double[] pos) {
        double angle = Utils.normalRelativeAngle(absoluteBearing(new Point2D.Double(pos[0], pos[1])) - directAngle) / maxEscapeAngle;
        if(angle < minFactor) minFactor = angle;
        if(angle > maxFactor) maxFactor = angle;
    }

    /**
     * Reset our factors for calculating position (important!)
     */
    public void resetFactors() {
        minFactor = java.lang.Double.POSITIVE_INFINITY;
        maxFactor = java.lang.Double.NEGATIVE_INFINITY;
        lastTarget.setLocation(0, 0);
    }
}

class BulletShadow {
    public Bullet b;
    public double[] shadow;

    public final boolean equals(Bullet q) {
        if(Math.abs(b.getHeadingRadians() - q.getHeadingRadians()) < 0.001 && Math.abs(b.getPower() - q.getPower()) < 0.001
                && Math.abs(b.getX()-q.getX()) < 0.001 && Math.abs(b.getY()-q.getY()) < 0.001) {
            return true;
        }
        return false;
    }
}

