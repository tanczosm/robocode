package ja;

import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Gun designed for flat targeting - only use if you want to see if your other guns are terrible
 */
public class FlatGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;
    public double[] firingAngles = new double[180];
    public int fireIndex = 0;

    public FlatGun(AdvancedRobot robot, RadarScanner radarScanner)
    {
        _robot = robot;
        _radarScanner = radarScanner;
        regenerateAngles ();
    }

    public String getName()
    {
        return "FLAT";
    }

    public void update()
    {

    }

    public void regenerateAngles ()
    {
        for (int i = 0; i < 180; i++)
        {
            firingAngles[i] = ((double)(i-90) * (Math.PI / 180d));
        }

        // Now randomize angles
        for (int i = 0; i < 180; i++)
        {
            int swapIndex = (int)(Math.random() * 180);
            double temp = firingAngles[i];
            firingAngles[i] = firingAngles[swapIndex];
            firingAngles[swapIndex] = temp;
        }
    }

    public double getNextBearing (double maxEscapeAngle)
    {

        while (Math.abs(firingAngles[fireIndex]) > maxEscapeAngle)
        {
            fireIndex++;

            if (fireIndex == firingAngles.length)
            {
                regenerateAngles();
                fireIndex = 0;
            }
        }
System.out.println(firingAngles[fireIndex] + " " + maxEscapeAngle);
        double retval = firingAngles[fireIndex];

        fireIndex++;
        if (fireIndex == firingAngles.length)
        {
            regenerateAngles();
            fireIndex = 0;
        }

        return retval;
    }

    public void addSituation(Situation s)
    {
        // Do nothing
    }

    public double projectBearing(Situation scan, double x, double y, double enemyHeading)
    {
        // Head-On Targeting
        double mea = CTUtils.maxEscapeAngle(CTUtils.bulletVelocity(_radarScanner.FIRE_POWER));

        return getNextBearing(mea/2);
    }
}
