package ja;


import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;

/**
 * Created by tanczosm on 6/2/2015.
 */
public class CircularGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;
    private double lastHeading= 0;

    public CircularGun(AdvancedRobot robot, RadarScanner radarScanner)
    {
        _robot = robot;
        _radarScanner = radarScanner;
    }

    public String getName()
    {
        return "CIRC";
    }

    public void update()
    {

    }

    public void addSituation(Situation s)
    {
        // Do nothing
    }


    public double projectBearing(Situation scan, double x, double y, double enemyHeading)
    {
        // Circular Targeting
        //double absoluteBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;
        //double heading = _radarScanner._lastScan.getHeadingRadians();
        double heading = enemyHeading;
        double velocity = _radarScanner._lastScan.getVelocity();
        double projectedTime = 0;


        double dX = _radarScanner.nme.location.getX(), dY = _radarScanner.nme.location.getY();
        double currentHeading = heading;
        double deltaHeading = heading - lastHeading; //_radarScanner._lastHeading;
        double bulletVelocity = CTUtils.bulletVelocity(RadarScanner.FIRE_POWER);

//System.out.println("ch: " + currentHeading + ", dH: " + deltaHeading + ", v: " + velocity + ",x:" + x + ",y: " + y + ",dx: " + dX + ",dy:" + dY );
        do {
            projectedTime++;
            dX += Math.sin(currentHeading) * velocity;
            dY += Math.cos(currentHeading) * velocity;
            currentHeading += deltaHeading;
        } while (Point2D.distance(x, y, dX, dY) > projectedTime * bulletVelocity && RadarScanner._fieldRect.contains(dX, dY));

        Graphics2D g = _robot.getGraphics();
        g.drawOval((int)dX, (int)dY, 2, 2);

        double angleOffset = Math.atan2(dX - x, dY - y) - enemyHeading ;
//Utils.normalRelativeAngle(
        //System.out.println("CIRC ANGLE: " + angleOffset);
        lastHeading = heading;

        return angleOffset;
    }

/*
    public double projectBearing(Situation scan, double x, double y, double enemyHeading) {
        double bulletPower = _radarScanner.FIRE_POWER;

        double absoluteBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        double enemyX = x + _radarScanner.nme.distance * Math.sin(absoluteBearing);
        double enemyY = y + _radarScanner.nme.distance * Math.cos(absoluteBearing);

        double enemyHeadingChange = enemyHeading - lastHeading;
        double enemyVelocity = _radarScanner.nme.velocity;

        double deltaTime = 0;
        double battleFieldHeight = _radarScanner._fieldRect.height,
                battleFieldWidth = _radarScanner._fieldRect.width;
        double predictedX = enemyX, predictedY = enemyY;
        while ((++deltaTime) * (20.0 - 3.0 * bulletPower) <
                Point2D.Double.distance(x, y, predictedX, predictedY)) {
            predictedX += Math.sin(enemyHeading) * enemyVelocity;
            predictedY += Math.cos(enemyHeading) * enemyVelocity;
            enemyHeading += enemyHeadingChange;
            if (predictedX < 18.0
                    || predictedY < 18.0
                    || predictedX > battleFieldWidth - 18.0
                    || predictedY > battleFieldHeight - 18.0) {

                predictedX = Math.min(Math.max(18.0, predictedX),
                        battleFieldWidth - 18.0);
                predictedY = Math.min(Math.max(18.0, predictedY),
                        battleFieldHeight - 18.0);
                break;
            }
        }
        double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - x, predictedY - y));
        Graphics2D g = _robot.getGraphics();
        g.drawOval((int)predictedX, (int)predictedY, 2, 2);

        lastHeading = enemyHeading;

        return absoluteBearing - theta - _robot.getGunHeadingRadians();
    }
*/
}
