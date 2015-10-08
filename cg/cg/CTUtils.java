package cg;

import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/**
 * Created by tanczosm on 5/29/2015.
 */
public class CTUtils {

    public static MovSim moveSimulator;

    // CREDIT: from CassiusClay, by PEZ
    //   - returns point length away from sourceLocation, at angle
    // robowiki.net?CassiusClay
    public static Point2D.Double project(Point2D.Double sourceLocation, double angle, double length) {
        return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
                sourceLocation.y + Math.cos(angle) * length);
    }

    // got this from RaikoMicro, by Jamougha, but I think it's used by many authors
    //  - returns the absolute angle (in radians) from source to target points
    public static double absoluteBearing(Point2D.Double source, Point2D.Double target) {
        return Math.atan2(target.x - source.x, target.y - source.y);
    }


    public static int bulletTicks(double distance, double power) {
        return (int) Math.ceil(distance / (20 - (3 * power)));
    }

    public static double bulletDamage(double power) {
        return (power * 4) + (2 * Math.max(power - 1, 0));
    }

    public static double bulletLifeGain(double power) {
        return (3 * power);
    }

    public static double gunHeat(double power) {
        return 1 + (power / 5);
    }

    public static double gunHeatTicks(double power, double coolingRate) {
        return Math.ceil((1 + (power / 5)) / coolingRate);
    }

    public static double botWidthAimAngle(double distance) {
        return Math.abs(18.0 / distance);
    }

    public static Point2D.Double nextLocation(AdvancedRobot robot) {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        MovSimStat[] next = moveSimulator.futurePos(1, robot);

        return new Point2D.Double(next[0].x, next[0].y);
    }

    public static MovSimStat nextLocationFull(AdvancedRobot robot) {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        MovSimStat[] next = moveSimulator.futurePos(1, robot);

        return next[0];
    }

    /*
        public static Point2D.Double nextLocation(BotScan scan) {
        return new Point2D.Double(scan.getLocation().x
            + (Math.sin(scan.getHeadingRadians())*scan.getVelocity()),
            scan.getLocation().y
            + (Math.cos(scan.getHeadingRadians())*scan.getVelocity()));
    }
     */

    public static MovSim getMovSim() {
        if (moveSimulator == null) {
            moveSimulator = new MovSim();
        }

        return moveSimulator;
    }

    public static int getOrientation(double targetHeadingRadians,
                                     double targetVelocity, double absBearingRadians) {

        double relativeHeadingRadians = Utils.normalRelativeAngle(
                targetHeadingRadians - absBearingRadians +
                        (targetVelocity < 0 ? Math.PI : 0));

        return sign(relativeHeadingRadians);
    }

    public static RobotState nextPerpendicularLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double enemyHeadingRadians,
            boolean clockwise, long currentTime, boolean ignoreWallHits) {

        int purelyPerpendicularOffset = 0;

        return nextPerpendicularLocation(targetLocation, absBearingRadians,
                enemyVelocity, enemyHeadingRadians, purelyPerpendicularOffset,
                clockwise, currentTime, ignoreWallHits);
    }

    public static RobotState nextPerpendicularLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double enemyHeadingRadians, double attackAngle,
            boolean clockwise, long currentTime, boolean ignoreWallHits) {

        return nextPerpendicularWallSmoothedLocation(targetLocation,
                absBearingRadians, enemyVelocity, 8.0, enemyHeadingRadians,
                attackAngle, clockwise, currentTime, null, 0, 0, 0,
                ignoreWallHits);
    }

    public static RobotState nextPerpendicularWallSmoothedLocation(
            Point2D.Double targetLocation, double absBearingRadians,
            double enemyVelocity, double maxVelocity, double enemyHeadingRadians,
            double attackAngle, boolean clockwise, long currentTime,
            Rectangle2D.Double battleField, double bfWidth, double bfHeight,
            double wallStick, boolean ignoreWallHits) {

        int orientation;
        if (clockwise) {
            orientation = 1;
        } else {
            orientation = -1;
        }

        double goAngleRadians = Utils.normalRelativeAngle(
                absBearingRadians + (orientation * ((Math.PI / 2) + attackAngle)));

        boolean isSmoothing = false;

        if (wallStick != 0 && battleField != null) {
            double smoothedAngle = CTUtils.wallSmoothing(
                    battleField, bfWidth, bfHeight, targetLocation,
                    goAngleRadians, orientation, wallStick);

            if (CTUtils.round(smoothedAngle, 4)
                    != CTUtils.round(goAngleRadians, 4)) {
                isSmoothing = true;
            }

            goAngleRadians = smoothedAngle;
        }

        return nextLocation(targetLocation, enemyVelocity, maxVelocity,
                enemyHeadingRadians, goAngleRadians, currentTime,
                isSmoothing, ignoreWallHits, bfWidth, bfHeight);
    }

    public static RobotState nextLocation(
            Point2D.Double targetLocation, double enemyVelocity, double maxVelocity,
            double enemyHeadingRadians, double goAngleRadians, long currentTime,
            boolean isSmoothing, boolean ignoreWallHits, double bfWidth, double bfHeight) {

        MovSim movSim = CTUtils.getMovSim();

        double futureTurn = Utils.normalRelativeAngle(
                goAngleRadians - enemyHeadingRadians);
        double futureDistance;

        if (Math.abs(futureTurn) > (Math.PI / 2)) {
            if (futureTurn < 0) {
                futureTurn = Math.PI + futureTurn;
            } else {
                futureTurn = -1 * (Math.PI - futureTurn);
            }

            futureDistance = -1000;
        } else {
            futureDistance = 1000;
        }

        int extraWallSize = 0;

        if (ignoreWallHits) {
            extraWallSize = 50000;
        }

        MovSimStat[] futureMoves = movSim.futurePos(
                1, extraWallSize + targetLocation.x,
                extraWallSize + targetLocation.y, enemyVelocity, maxVelocity,
                enemyHeadingRadians, futureDistance,
                futureTurn, 10.0, extraWallSize * 2 + bfWidth,
                extraWallSize * 2 + bfHeight);

        return new RobotState(
                new Point2D.Double(CTUtils.round(futureMoves[0].x - extraWallSize, 3),
                        CTUtils.round(futureMoves[0].y - extraWallSize, 3)), futureMoves[0].h,
                futureMoves[0].v, currentTime + 1, isSmoothing);
    }

    public static double round(double d, int i) {
        long powerTen = 1;

        for (int x = 0; x < i; x++) {
            powerTen *= 10;
        }

        return ((double) Math.round(d * powerTen)) / powerTen;
    }

    /**
     * wallSmoothing: do some Voodoo and wall smooth in a very efficiently.
     * - ...in terms of CPU cycles, not amount of code.
     * - used to be iterative, which was a lot simpler and more readable,
     * but far too slow with how much it was called during precise
     * prediction.
     */
    public static double wallSmoothing(Rectangle2D.Double field, double bfWidth,
                                       double bfHeight, Point2D.Double startLocation, double startAngleRadians,
                                       int orientation, double wallStick) {

/*
        double angle = startAngle;
        _lastWallSmoothAway = false;
        while (!field.contains(x + Math.sin(Math.toRadians(angle))*WALL_STICK,
            y+Math.cos(Math.toRadians(angle))*WALL_STICK)) {
            angle += orientation*smoothNormal*7.0;
            if (smoothNormal == -1) { _lastWallSmoothAway = true; }
        }

        return angle;
*/
        // Trying to do almost exactly the equivalent of the above in more
        // code but less CPU time. The above needs a low increment to work
        // perfectly smoothly, which results in very slow execution.
        //
        // NOTE: The two algorithms can give slightly different results,
        //       but that is mainly because the iterative one never tests a
        //       very specific angle in a corner that would turn up "in bounds";
        //       if it increased the angle var by (1/INFINITY), they'd be the
        //       same (as far as I can tell.)

        double angle = startAngleRadians;
        double wallDistanceX = Math.min(startLocation.x - 18,
                bfWidth - startLocation.x - 18);
        double wallDistanceY = Math.min(startLocation.y - 18,
                bfHeight - startLocation.y - 18);

        if (wallDistanceX > wallStick && wallDistanceY > wallStick) {
            return startAngleRadians;
        }

        double testX = startLocation.x + (Math.sin(angle) * wallStick);
        double testY = startLocation.y + (Math.cos(angle) * wallStick);
        double testDistanceX = Math.min(testX - 18, bfWidth - testX - 18);
        double testDistanceY = Math.min(testY - 18, bfHeight - testY - 18);

        double adjacent = 0;
        int g = 0;

        while (!field.contains(testX, testY) && g++ < 25) {
            if (angle < 0) {
                angle += (2 * Math.PI);
            }
            if (testDistanceY < 0 && testDistanceY < testDistanceX) {
                // wall smooth North or South wall
                angle = ((int) ((angle + (Math.PI / 2)) / Math.PI)) * Math.PI;
                adjacent = Math.abs(wallDistanceY);
            } else if (testDistanceX < 0 && testDistanceX <= testDistanceY) {
                // wall smooth East or West wall
                angle = (((int) (angle / Math.PI)) * Math.PI) + (Math.PI / 2);
                adjacent = Math.abs(wallDistanceX);
            }

            angle += orientation *
                    (Math.abs(Math.acos(adjacent / wallStick)) + 0.0005);

            testX = startLocation.x + (Math.sin(angle) * wallStick);
            testY = startLocation.y + (Math.cos(angle) * wallStick);
            testDistanceX = Math.min(testX - 18, bfWidth - testX - 18);
            testDistanceY = Math.min(testY - 18, bfHeight - testY - 18);
        }

        return angle;
    }

    public static int sign(double p) {
        return p < 0 ? -1 : 1;
    }

    public static double limit(double min, double value, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double bulletVelocity(double power) {
        return (20D - (3D * power));
    }

    public static double maxEscapeAngle(double velocity) {
        return Math.asin(8.0 / velocity);
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static void setBackAsFront(AdvancedRobot robot, double goAngle) {
        double angle =
                Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
        if (Math.abs(angle) > (Math.PI / 2)) {
            if (angle < 0) {
                robot.setTurnRightRadians(Math.PI + angle);
            } else {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(Double.POSITIVE_INFINITY);
        } else {
            if (angle < 0) {
                robot.setTurnLeftRadians(-1 * angle);
            } else {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(Double.POSITIVE_INFINITY); //Math.cos(angle) * Double.POSITIVE_INFINITY);
        }
    }

}
