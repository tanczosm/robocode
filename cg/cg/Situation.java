package cg;

import robocode.*;
import robocode.util.Utils;

// See Chalk bot source for calculations
public class Situation {

    // How we will segment the tree
    public static final int LAT_VEL = 0;
    public static final int ACCEL = 1;
    public static final int VEL_CHNG = 2;
    public static final int DIST = 3;
    public static final int WALL_F = 4;
    public static final int WALL_B = 5;
    public static final int ADV_VEL = 6;

    public long Time;
    public double RX;
    public double RY;
    public double EnemyHeading;
    public double Bearing;
    public double BearingRadians;
    public double NormalizedDistance;
    public double Distance;
    public double DistanceDelta;
    public double DistanceLast10;
    public double PlayerLateralVelocity;
    public double LateralVelocity;
    public double AdvancingVelocity;
    public double Acceleration;
    public double Velocity;
    public double SinceVelocityChange;
    public double WallTriesForward;
    public double WallTriesBack;
    public double Direction;
    public double BulletVelocity;
    public double MaxAngle;
    public double GuessFactor;
    public double GuessFactorChosen;
    public boolean Set = false;
    public boolean DeltaSet = false;

    public void setBulletVelocity(double shotPower) {
        BulletVelocity = 20d - 3d * shotPower;
        MaxAngle = Math.asin(8d / BulletVelocity) * Direction;
    }

    public double getDistance(long time) {
        return (double) (time - Time) * BulletVelocity;
    }

    public boolean setBearing(double x, double y) {
        boolean val = Set;
        if (!Set) {
            register(x, y);
        }
        if (!DeltaSet) {
            DistanceDelta = Math.sqrt(Math.pow(RX - x, 2) + Math.pow(RY - y, 2)) - Distance;
            DeltaSet = true;
        }
        return val;
    }

    public void registerHit(double x, double y) {
        register(x, y);
    }

    private void register(double x, double y) {
        BearingRadians = Utils.normalRelativeAngle(Math.atan2(x - RX, y - RY) - EnemyHeading);
        Bearing = (BearingRadians / MaxAngle) * 100d;

        GuessFactor = Math.max(-1, Math.min(1, BearingRadians / MaxAngle)) * Direction;


        Set = true;
    }

    /*
     *    Math.sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
     */
    public double[] getPoint() {
        return new double[]{LateralVelocity, Acceleration, NormalizedDistance, WallTriesForward, WallTriesBack, AdvancingVelocity};
    }

    public double getProperty(int index) {
        switch (index) {
            case LAT_VEL:
                return LateralVelocity;
            case ACCEL:
                return Acceleration;
            case VEL_CHNG:
                return SinceVelocityChange;
            case DIST:
                return NormalizedDistance;
            case WALL_F:
                return WallTriesForward;
            case WALL_B:
                return WallTriesBack;
            case ADV_VEL:
                return AdvancingVelocity;
        }
        return 0.0d;
    }
}
