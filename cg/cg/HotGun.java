package cg;

import robocode.AdvancedRobot;

/**
 * Gun designed for head-on targeting - only used as a last option if enemy is disabled
 */
public class HotGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    public HotGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;
    }

    public String getName() {
        return "HOT";
    }

    public void onRoundEnded()
    {

    }

    public void update() {

    }

    public void addSituation(Situation s) {
        // Do nothing
    }

    public double projectBearing(Situation scan, double x, double y, double enemyHeading) {
        // Head-On Targeting
        double absoluteBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        return 0;
    }
}
