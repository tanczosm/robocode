package cg;

import robocode.*;

import java.awt.geom.*;

import robocode.util.Utils;

public class NNBullet {
    public double startX, startY, startBearing, power, directAngle, maxEscapeAngle;
    public long fireTime;
    public double direction;
    public double[] outputs;
    public double[] inputs;
    public int fireIndex;

    public double lowGF, highGF;

    public NNBullet(double x, double y, double bearing, double directAngle, double power,
                      int direction, long time, double[] inputs) {
        startX = x;
        startY = y;
        startBearing = bearing;
        this.power = power;
        this.directAngle = directAngle;
        this.direction = direction;
        this.inputs = inputs;
        fireTime = time;
        outputs = null;
        maxEscapeAngle = CTUtils.maxEscapeAngle(CTUtils.bulletVelocity(power));
        fireIndex = GFGun.GF_ZERO;

        lowGF = -1d;
        highGF = 1d;
        //System.out.println("Firing with max escape angle of " + maxEscapeAngle);
        //System.out.println("Start bearing: " + bearing);
    }

    public Point2D.Double getCurrentLocation(int time) {

        return CTUtils.project(new Point2D.Double(startX, startY), startBearing, CTUtils.bulletVelocity(power) * (time - fireTime));
    }

    public double getGuessFactor(double enemyX, double enemyY) {
        double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
        double angleOffset = (Utils.normalRelativeAngle(desiredDirection - startBearing));
        double guessFactor = angleOffset / maxEscapeAngle * direction;
        return guessFactor;
    }

    public boolean checkHit(double enemyX, double enemyY, long currentTime) {
        double bulletVelocity = getBulletSpeed();


        // if the distance from the wave origin to our enemy has passed
        // the distance the bullet would have traveled...
        if (((currentTime - fireTime) * bulletVelocity) > Point2D.distance(startX, startY, enemyX, enemyY)) {
            /*
            double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
            double angleOffset = Utils.normalRelativeAngle(desiredDirection - startBearing);
            double guessFactor =
                    Math.max(-1, Math.min(1, angleOffset / maxEscapeAngle)) * direction;
            int index = (int) Math.round((double)GFGun.GF_ZERO * (guessFactor + 1));
            //System.out.println("saving to index " + index);
            System.out.println("desiredDirection: " + desiredDirection + ", angleOffset= " + angleOffset + ", GF= " + guessFactor);
            */
            /*
            double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
            double angleOffset = (Utils.normalRelativeAngle(desiredDirection - startBearing));
            double guessFactor = angleOffset / maxEscapeAngle * direction;
            int index = (int)Math.round((guessFactor+1) * GFGun.GF_ZERO);
            System.out.println("startBearing: " + startBearing + ", desiredDirection: " + desiredDirection + ", direction= " + direction + ", angleOffset= " + angleOffset + ", GF= " + guessFactor + ", maxEscapeAngle= " + maxEscapeAngle);
            */
            double gf = getGuessFactor(enemyX, enemyY);

            double[] centers = RBFUtils.getCenters(-1.0, 1.0, 61);

            outputs = RBFUtils.processDataIntoFeatures(gf, 0.25, centers);

            //waveGuessFactors[(int)Math.round((Utils.normalRelativeAngle(absoluteBearing(firePosition, RaikoGun.enemyLocation) - enemyAbsBearing))/bearingDirection + GF_ZERO)]++;
            /*
            double max = 0;
            for (int x = 0; x < returnSegment.length; x++) {
                returnSegment[x] += Math.pow(0.8, Math.abs(x - index));
                if (returnSegment[x] > max)
                    max = returnSegment[x];
            }

            for (int x = 0; x < returnSegment.length; x++) {
                returnSegment[x] /= (max / 4);
            }
            */
            return true;
        }
        return false;
    }

    public double getBulletSpeed() {
        return 20 - power * 3;
    }

} // end WaveBullet class
