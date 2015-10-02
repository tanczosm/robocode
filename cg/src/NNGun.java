package cg;

import robocode.AdvancedRobot;

import java.util.ArrayList;
import java.util.List;

/**
 * Gun designed for neural network-based targeting
 */
public class NNGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    public static final int GF_ZERO = 30; // 23; //15;
    public static final int GF_ONE = 60; //46; //30;

    private List<NNBullet> waves = new ArrayList<NNBullet>();
    private NNBullet newWave = null;
    int direction = 1;

    public NNGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;
    }

    public String getName() {
        return "NN";
    }

    public void update() {

        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        // Let's process the waves now:
        for (int i = 0; i < waves.size(); i++) {
            NNBullet currentWave = (NNBullet) waves.get(i);

            if (currentWave.checkHit(ex, ey, _robot.getTime())) {

                // Train based on waves.returnSegment

                waves.remove(currentWave);
                i--;
            }

        }
    }

    public void takeVirtualShot(Situation s, double bearing) {
        super.takeVirtualShot(s, bearing);

        //newWave.startBearing = bearing; // Update bearing to actual bearing..


        if (newWave != null)
            waves.add(newWave);

    }

    public void addSituation(Situation s) {
        // Do nothing
    }

    public double projectBearing(Situation s, double x, double y, double enemyHeading) {

        if (s == null)
            return 0;

        //double[] currentStats = getStats(s); //stats[(int)(s.Distance / 100)];
        double absBearing = enemyHeading; //_robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // don't try to figure out the direction they're moving
        // they're not moving, just use the direction we had before
        if (_radarScanner.nme.velocity != 0) {
            if (Math.sin(_radarScanner.nme.headingRadians - absBearing) * _radarScanner.nme.velocity < 0)
                direction = -1;
            else
                direction = 1;
        }
        //w.bearingDirection = (double)direction *Math.asin(8D/newWave.getBulletSpeed())/GF_ZERO;

        newWave = new NNBullet(x, y, absBearing, ((Double) _radarScanner._surfAbsBearings.get(0)).doubleValue(), _radarScanner.FIRE_POWER,
                direction, _robot.getTime());


        int bestGF = GF_ZERO;    // initialize it to be in the middle, guessfactor 0.

        // TODO: USE NEURAL NET TO FIND BETTER GF

        newWave.fireIndex = bestGF;

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double) (bestGF - GF_ZERO) / (double) GF_ZERO;

        /*
        if (_robot.getTime()/5 % 5 == 0)
        {
            guessfactor = (guessfactor+newWave.highGF)/2;
        }
        if (_robot.getTime()/5 % 5 == 1)
        {
            guessfactor = (guessfactor+newWave.lowGF)/2;
        }
        */


        //double angleOffset = CTUtils.clamp(direction * guessfactor, newWave.lowGF, newWave.highGF) * newWave.maxEscapeAngle;


        double angleOffset = direction < 0 ? -(guessfactor * newWave.lowGF * newWave.maxEscapeAngle) :
                (guessfactor * newWave.highGF * newWave.maxEscapeAngle);

//                CTUtils.clamp(direction * guessfactor, newWave.lowGF, newWave.highGF) * newWave.maxEscapeAngle;

        return angleOffset;


    }
}
