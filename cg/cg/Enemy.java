package cg;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Enemy {

    public Point2D.Double location;
    public Point2D.Double lastlocation;
    public Rectangle2D.Double enemyBox;
    public String name;
    public double energy;
    public double lastShotTime;
    public double lastBulletPower;
    public double distance;
    public double bearing;
    public double bearingRadians;
    public double heading;
    public double headingRadians;
    public double velocity;
    public double enemyShotHits = 0;
    public double enemyShotMisses = 0;

    public static ArrayList<EnemyWave> waves;

    public Enemy() {
        energy = 100;
        lastShotTime = 0;
        enemyBox = new Rectangle2D.Double();
        lastBulletPower = 2;
    }

    public double getRatingPercent() {
        return (double) (enemyShotHits) / (enemyShotHits + enemyShotMisses);
    }

    /*
    public static double getLastBulletPower(double defaultPower) {
        if (Enemy.waves == null || Enemy.waves.size() == 0)
            return defaultPower;

        return Enemy.waves.get(waves.size() - 1).bulletPower;
    }
    */

}
