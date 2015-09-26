package enemy;

import robocode.AdvancedRobot;
import robocode.HitRobotEvent;
import robocode.ScannedRobotEvent;
import robocode.HitWallEvent;

import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;

public class CirclingBot extends AdvancedRobot {

    boolean movingForward;
    boolean inWall;

    public void run() {

        setAllColors(Color.green);

        setAdjustRadarForRobotTurn(true);
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
            inWall = true;
        } else {
            inWall = false;
        }

        setAhead(Double.POSITIVE_INFINITY);
        setTurnRadarRight(360);
        movingForward = true;

        while (true) {
            if (getX() > 50 && getY() > 50 && getBattleFieldWidth() - getX() > 50 && getBattleFieldHeight() - getY() > 50 && inWall == true) {
                inWall = false;
            }
            if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
                if (inWall == false) {
                    reverseDirection();
                    inWall = true;
                }
            }

            if (getRadarTurnRemaining() == 0.0) {
                setTurnRadarRight(360);
            }

            execute();
        }
    }

    public void onHitWall(HitWallEvent e) {
        reverseDirection();
    }

    public void reverseDirection() {
        if (movingForward) {
            setBack(Double.POSITIVE_INFINITY);
            movingForward = false;
        } else {
            setAhead(Double.POSITIVE_INFINITY);
            movingForward = true;
        }
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        double absoluteBearing = getHeading() + e.getBearing();
        double bearingFromGun = normalRelativeAngleDegrees(absoluteBearing - getGunHeading());
        double bearingFromRadar = normalRelativeAngleDegrees(absoluteBearing - getRadarHeading());


        if (movingForward) {
            setTurnRight(normalRelativeAngleDegrees(e.getBearing() + 80));
        } else {
            setTurnRight(normalRelativeAngleDegrees(e.getBearing() + 100));
        }
        /*
        if (Math.abs(bearingFromGun) <= 4) {
            setTurnGunRight(bearingFromGun);
            setTurnRadarRight(bearingFromRadar);

            if (getGunHeat() == 0 && getEnergy() > .2) {
                fire(Math.min(4.5 - Math.abs(bearingFromGun) / 2 - e.getDistance() / 250, getEnergy() - .1));
            }
        }
        */
        setTurnGunRight(bearingFromGun);
        setTurnRadarRight(bearingFromRadar);

        if (bearingFromGun == 0) {
            scan();
        }
    }

    public void onHitRobot(HitRobotEvent e) {
        if (e.isMyFault()) {
            reverseDirection();
        }
    }
}