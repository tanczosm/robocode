package cg;

import robocode.util.Utils;

import java.awt.geom.Point2D;
import java.util.Arrays;

public class NNBullet {
    public double startX, startY, startBearing, power, directAngle, maxEscapeAngle;
    public Point2D.Double fireLocation;
    public long fireTime;
    public double direction;
    public double[] outputs;
    public double[] inputs;
    public boolean isReal = false;
    public boolean actualHit = false;
    public int fireIndex;

    public RobotState[] moves;
    public double lowGF, highGF;

    public NNBullet(double x, double y, double bearing, double directAngle, double power,
                    int direction, long time, double[] inputs) {
        startX = x;
        startY = y;
        fireLocation = new Point2D.Double(startX, startY);
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
            //System.out.println("Correct gf is " + gf);
            double[] centers = RBFUtils.getCenters(-1.0, 1.0, 61);

            // Calculate the effective bot width at distance as that will be needed to regulate
            // the width of the features
            /*
            double gf1 = getGuessFactor(enemyX-18, enemyY-18);
            double gf2 = getGuessFactor(enemyX-18, enemyY+18);
            double gf3 = getGuessFactor(enemyX+18, enemyY-18);
            double gf4 = getGuessFactor(enemyX+18, enemyY+18);

            double gfmin = Math.min(Math.min(gf1, gf2), Math.min(gf3, gf4));
            double gfmax = Math.max(Math.max(gf1, gf2), Math.max(gf3, gf4));
            double gfwidth = (gfmax-gfmin) * 0.25;
*/
            double gfwidth = CTUtils.botWidthAimAngle(Math.sqrt((enemyX-startX)*(enemyX-startX) + (enemyY-startY)*(enemyY-startY) ));

            //gfwidth = 0.05;
            //System.out.println("gfminmax: " + gfmin + ", " + gfmax + ", diff: " + gfwidth);
            //System.out.println("gfwidth: " + gfwidth);

            outputs = RBFUtils.processDataIntoFeatures(gf, gfwidth, centers);
            //outputs = RBFUtils.processDataIntoFeatures(gf, 0.05, centers);
            //System.out.println("Output gf is " + Arrays.toString(outputs));

            if (getCurrentLocation((int) currentTime).distance(enemyX, enemyY) <= 18) {
                actualHit = true;

                // System.out.println("Inverting outputs");
                // Invert the outputs to battle wave surfers more effectively (?)
                for (int i = 0; i < outputs.length; i++)
                {
                    //outputs[i] = 1.0 - outputs[i];
                }

            }

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
