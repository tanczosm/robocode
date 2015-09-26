package cg;

import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;

import java.awt.Color;

import static robocode.util.Utils.normalRelativeAngleDegrees;

public class V extends AdvancedRobot {

    public void run() {

        setAllColors(Color.black);

        while (true) {
            turnGunRight(Double.POSITIVE_INFINITY);
        }
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        double absoluteBearing = getHeading() + e.getBearing();
        double bearingFromGun = normalRelativeAngleDegrees(absoluteBearing - getGunHeading());

        setTurnGunRight(bearingFromGun);
    }
}