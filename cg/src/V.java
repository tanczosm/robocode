package cg;

import robocode.AdvancedRobot;
import robocode.HitRobotEvent;
import robocode.ScannedRobotEvent;
import robocode.HitWallEvent;

import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;

public class V extends AdvancedRobot {

    boolean movingForward; // Is set to true when setAhead is called, set to false on setBack
    boolean inWall; // Is true when robot is near the wall.

    public void run() {

        setAllColors(Color.black);

        setAdjustRadarForRobotTurn(true);
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
            inWall = true;
        } else {
            inWall = false;
        }

        setAhead(40000); // go ahead until you get commanded to do differently
        setTurnRadarRight(360); // scan until you find your first enemy
        movingForward = true; // we called setAhead, so movingForward is true

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

            // If the radar stopped turning, take a scan of the whole field until we find a new enemy
            if (getRadarTurnRemaining() == 0.0) {
                setTurnRadarRight(360);
            }

            execute(); // execute all actions set.
        }
    }

    public void onHitWall(HitWallEvent e) {
        // Bounce off!
        reverseDirection();
    }

    public void reverseDirection() {
        if (movingForward) {
            setBack(40000);
            movingForward = false;
        } else {
            setAhead(40000);
            movingForward = true;
        }
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        // Calculate exact location of the robot
        double absoluteBearing = getHeading() + e.getBearing();
        double bearingFromGun = normalRelativeAngleDegrees(absoluteBearing - getGunHeading());
        double bearingFromRadar = normalRelativeAngleDegrees(absoluteBearing - getRadarHeading());

        //Spiral around our enemy. 90 degrees would be circling it (parallel at all times)
        // 80 and 100 make that we move a bit closer every turn.
        if (movingForward) {
            setTurnRight(normalRelativeAngleDegrees(e.getBearing() + 80));
        } else {
            setTurnRight(normalRelativeAngleDegrees(e.getBearing() + 100));
        }


        // If it's close enough, fire!
        if (Math.abs(bearingFromGun) <= 4) {
            setTurnGunRight(bearingFromGun);
            setTurnRadarRight(bearingFromRadar); // keep the radar focussed on the enemy
            // We check gun heat here, because calling fire()
            // uses a turn, which could cause us to lose track
            // of the other robot.

            // The close the enmy robot, the bigger the bullet.
            // The more precisely aimed, the bigger the bullet.
            // Don't fire us into disability, always save .1
            if (getGunHeat() == 0 && getEnergy() > .2) {
                fire(Math.min(4.5 - Math.abs(bearingFromGun) / 2 - e.getDistance() / 250, getEnergy() - .1));
            }
        } // otherwise just set the gun to turn.

        else {
            setTurnGunRight(bearingFromGun);
            setTurnRadarRight(bearingFromRadar);
        }

        // Generates another scan event if we see a robot.
        // We only need to call this if the radar
        // is not turning.  Otherwise, scan is called automatically.
        if (bearingFromGun == 0) {
            scan();
        }
    }

    public void onHitRobot(HitRobotEvent e) {
        // If we're moving the other robot, reverse!
        if (e.isMyFault()) {
            reverseDirection();
        }
    }
}