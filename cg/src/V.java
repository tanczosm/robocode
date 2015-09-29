package cg;

import robocode.AdvancedRobot;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

import java.awt.Color;

import static robocode.util.Utils.normalRelativeAngleDegrees;

public class V extends AdvancedRobot {

    private boolean turnRadar = true;

    public void run() {

        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        //setAllColors(Color.black);
        turnRadarRightRadians(Double.POSITIVE_INFINITY);

        while (true) {
            turnGunRight(Double.POSITIVE_INFINITY);
        }
    }

    public void onScannedRobot(ScannedRobotEvent e) {

        double absBearing = e.getBearingRadians() + this.getHeadingRadians();

        setTurnGunRight(normalRelativeAngleDegrees(getHeading() + e.getBearing() - getGunHeading()));

        if (turnRadar) {
            double radarTurn = Utils.normalRelativeAngle(absBearing - this.getRadarHeadingRadians());
            double extraTurn = Math.min(Math.atan(44.0 / e.getDistance()), Rules.RADAR_TURN_RATE_RADIANS);
            radarTurn += (radarTurn < 0 ? -extraTurn : extraTurn);

            this.setTurnRadarRightRadians(radarTurn);
        }
    }
}