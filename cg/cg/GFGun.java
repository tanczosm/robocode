package cg;

import cg.BaseGun;
import cg.RadarScanner;
import cg.Situation;
import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

/**
 * Gun designed for head-on targeting - only used as a last option if enemy is disabled
 */
public class GFGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;
    private List<WaveBullet> waves = new ArrayList<WaveBullet>();
    private Situation _lastSituation;

    public static final int GF_ZERO = 45; // 23; //15;
    public static final int GF_ONE = 90; //46; //30;

    private static double[][][][][][] stats = new double[3][5][3][3][13][GF_ONE + 1]; // 31 is the number of unique GuessFactors we're using
    // Note: this must be odd number so we can get
    // GuessFactor 0 at middle.
    int direction = 1;
    private WaveBullet newWave = null;

    public GFGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;
        _lastSituation = null;
    }

    public String getName() {
        return "GF";
    }

    public void update() {
// Enemy absolute bearing, you can use your one if you already declare it.
        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        Graphics2D g = _robot.getGraphics();
        g.setColor(Color.WHITE);
        g.drawRect((int) ex - 32, (int) ey - 32, 64, 64);

        drawWaves();


        // Let's process the waves now:
        for (int i = 0; i < waves.size(); i++) {
            WaveBullet currentWave = (WaveBullet) waves.get(i);
            if (currentWave.checkHit(ex, ey, _robot.getTime())) {
                g.drawOval((int) currentWave.startX, (int) currentWave.startY, 4, 4);
                waves.remove(currentWave);
                i--;
            }

            Point2D.Double pt = currentWave.getCurrentLocation((int) _robot.getTime());
            Point2D.Double pt2 = CTUtils.project(pt, currentWave.startBearing, currentWave.getBulletSpeed());
            //g.drawOval((int)pt.getX()-3, (int)pt.getY()-3,6,6);
            g.setColor(new Color(0.0f, 1.0f, 1.0f, 0.5f));
            g.drawLine((int) pt.getX(), (int) pt.getY(), (int) pt2.getX(), (int) pt2.getY());
        }
    }

    public void addSituation(Situation s) {
        _lastSituation = s;
    }

    public void takeVirtualShot(Situation s, double bearing) {
        super.takeVirtualShot(s, bearing);

        //newWave.startBearing = bearing; // Update bearing to actual bearing..


        if (newWave != null)
            waves.add(newWave);

    }

    public static class GFIndex {
        public int accelIndex;  // 3 dimensions
        public int bestGF; // 5 dimensions
        public int vIndex; // 3 dimensions
        public int nearWall; // 5 dimensions
        public int distanceIndex;  // 13 dimensions

        public GFIndex(Situation s, AdvancedRobot _robot) {
            accelIndex = (int) Math.round(Math.abs(s.LateralVelocity * 8) - Math.abs(s.PlayerLateralVelocity * 8));
            double bearingDirection = 0;

            if (s.LateralVelocity * 8 != 0)
                bearingDirection = s.LateralVelocity * 8 > 0 ? 1 : -1;

            double moveTime = s.BulletVelocity * (s.SinceVelocityChange * 4) / s.Distance;
            bestGF = moveTime < .1 ? 1 : moveTime < .3 ? 2 : moveTime < 1 ? 3 : 4;

            vIndex = (int) Math.abs((s.LateralVelocity * 8) / 3);

            if (Math.abs(Math.abs(s.Velocity * 8) - Math.abs(_robot.getVelocity())) > .6) {

                //lastVChangeTime = 0;
                bestGF = 0;
                accelIndex = (int) Math.round(Math.abs(s.Velocity * 8) - Math.abs(_robot.getVelocity()));
                vIndex = (int) Math.abs((s.Velocity * 8) / 3);
            }

            if (accelIndex != 0)
                accelIndex = accelIndex > 0 ? 1 : 2;

            nearWall = (int) (((s.WallTriesBack + s.WallTriesForward) / 2d) * 2d); // 1 is totally clear of walls, closer to zero is near two walls (corner)
            /*
if (BF.contains(projectMotion(robotLocation, enemyAbsoluteBearing + w.bearingDirection * GF_ZERO, enemyDistance)))
    0
else if BF.contains(projectMotion(robotLocation, enemyAbsoluteBearing + .5 * w.bearingDirection * GF_ZERO, enemyDistance))
    1
else
    2
             */
            distanceIndex = (int) (s.Distance / 100);
        /*
        1.  accelIndex
        2.  bestGF
        3.  vIndex
        4.  Near wall segment (0 if wall
        5.  distanceIndex
        */
        }
    }

    public double[] getStats(Situation s) {
        GFIndex index = new GFIndex(s, _robot);
        //System.out.println("accelIndex: " + index.accelIndex + ", bestGF: " + index.bestGF + ", vIndex: " + index.vIndex + ", nearWall: " + index.nearWall + ", distance: " + index.distanceIndex);

        return stats[index.accelIndex][index.bestGF][index.vIndex][index.nearWall][index.distanceIndex];
        //return stats[0][0][0][0][0];
    }

    public RobotState[] calculateEnemyMoves(boolean clockwise)  // May want to make the absBearing to be perp to player always
    {
        double maxVelocity = 8;
        double heading = _radarScanner.nme.headingRadians;
        double velocity = _radarScanner.nme.velocity;
        double absBearingRadians = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;
        double x = _radarScanner.nme.location.getX();
        double y = _radarScanner.nme.location.getY();
        double attackAngle = 0; // Assume the most field coverage?
        int currentTime = (int) _robot.getTime();

        /*
        boolean clockwise = false;

        // they're not moving, just use the direction we had before
        if ( _radarScanner.nme.velocity != 0)
        {
            if (Math.sin(_radarScanner.nme.headingRadians - absBearingRadians)*_radarScanner.nme.velocity < 0)
                clockwise = false;
            else
                clockwise = true;
        }
        */

        double maxEscapeAngle = CTUtils.maxEscapeAngle(_radarScanner.FIRE_SPEED);

        int possibleTicks = CTUtils.bulletTicks(_radarScanner.nme.distance + 32, _radarScanner.FIRE_POWER);
        int movesToFind = possibleTicks; //(int)CTUtils.clamp(3*possibleTicks, 20, 50); // Presently 3 and 50 are optimal

        Graphics2D g = _robot.getGraphics();
        RobotState[] moves = new RobotState[movesToFind];

        for (int i = 0; i < movesToFind; i++) {
            RobotState next = CTUtils.nextPerpendicularWallSmoothedLocation(
                    new Point2D.Double(x, y), absBearingRadians, velocity, maxVelocity, heading,
                    attackAngle, clockwise, currentTime,
                    _radarScanner._fieldRect, 800, 600,
                    140, false);

            moves[i] = next;
            x = next.location.getX();
            y = next.location.getY();
            velocity = next.velocity;
            heading = next.heading;
            absBearingRadians = CTUtils.absoluteBearing(_radarScanner._myLocation, next.location); // Location is probably relative to last shot

            g.setColor(new Color(0.36078432f, 0.56078434f, 1.0f));
            g.drawOval((int) x, (int) y, 2, 2);
        }

        return moves;
    }

    public RobotState[] getEnemyMoves() {
        RobotState[] right = calculateEnemyMoves(true);
        RobotState[] left = calculateEnemyMoves(false);

        RobotState[] moves = new RobotState[left.length + right.length];
        int middleIndex = left.length;

        int j = 0;
        for (int i = left.length - 1; i >= 0; i--) {
            moves[j++] = left[i];

        }

        for (int i = 0; i < right.length; i++) {
            moves[i + middleIndex] = right[i];
        }

        return moves;

    }

    public void drawWaves() {
        Graphics2D g = _robot.getGraphics();

        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;


        //System.out.println("LOW BEARING: " + lowBearing + " HIGH BEARING: " + highBearing + " ABSBEARING: " + Utils.normalAbsoluteAngle(absBearing));

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        double enemyHeading = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians; // smallOffset + (_radarScanner._lastScan == null ? 0 : _radarScanner.nme.bearingRadians);

        double graphWidth = 300;
        int graphx = 30, graphy = 100, cnt = 0; //(int)_robot.getBattleFieldWidth()-((int)graphWidth+30), graphy=100, cnt=0;

        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy, graphx + ((int) graphWidth), graphy);
        g.drawLine(graphx, graphy, graphx, graphy + 100);
        g.drawLine(graphx + ((int) graphWidth), graphy, graphx + (int) graphWidth, graphy + 100);


        if (waves.size() > 0) {

            double bestdist = Double.MAX_VALUE;
            int minIndex = 0;
            int cTime = (int) _robot.getTime();

            for (int i = 0; i < waves.size(); i++) {
                WaveBullet w = waves.get(i);
                double dist = CTUtils.bulletVelocity(w.power) * (cTime - w.fireTime);

                if (dist > bestdist && dist < 1000) {
                    minIndex = i;
                    bestdist = dist;
                }

            }
            WaveBullet closest = waves.get(minIndex);


            RobotState[] moves = closest.moves;
            Point2D.Double fireLocation = new Point2D.Double(closest.startX, closest.startY);

            double lowBearing = CTUtils.absoluteBearing(fireLocation, moves[0].location);
            double highBearing = CTUtils.absoluteBearing(fireLocation, moves[moves.length - 1].location);

            double lowOffsetAngle = lowBearing - absBearing;
            double highOffsetAngle = highBearing - absBearing;

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            Point2D.Double low = CTUtils.project(fireLocation, lowBearing, _radarScanner.nme.distance);
            g.drawLine((int) closest.startX, (int) closest.startY, (int) low.getX(), (int) low.getY());

            Point2D.Double high = CTUtils.project(fireLocation, highBearing, _radarScanner.nme.distance);
            g.drawLine((int) closest.startX, (int) closest.startY, (int) high.getX(), (int) high.getY());


            int bestDensityIndex = 0;
            int lowestDensityIndex = 0;
            int lowestGFIndex = GF_ZERO;
            double bestDensity = -Double.MAX_VALUE, lowestDensity = Double.MAX_VALUE;

            for (int i = closest.returnSegment.length; --i >= 0; ) {

                double density = 0;
                double u;
                for (int j = closest.returnSegment.length; --j >= 0; ) {
                    //double kdeDiff = points[i].danger - points[j].danger;
                    double kdeDiff = closest.returnSegment[j] - closest.returnSegment[i];
                    //double kdeDiff = Math.sqrt(Math.pow(points[i].danger - points[j].danger,2));
                    //double kdeDiff = Math.abs(bestNodes.get(i).Distance - bestNodes.get(j).Distance);

                    density += Math.exp(
                            (
                                    u = (kdeDiff) /// 2 // BAND_WIDTH OF 4 seems ok
                            )
                                    * u
                                    * -0.5d
                    );
                }

                if (closest.returnSegment[i] < closest.returnSegment[lowestGFIndex])
                    lowestGFIndex = i;

                if (density > bestDensity) {
                    bestDensityIndex = i;
                    bestDensity = density;
                }

                if (density < lowestDensity) {
                    lowestDensityIndex = i;
                    lowestDensity = density;
                }

            }


            int plotpt = graphx + (int) (((double) closest.lowGF + 1) / 2.0 * graphWidth);

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            g.drawLine(plotpt, graphy, plotpt, graphy + 100);

            plotpt = graphx + (int) (((double) closest.highGF + 1) / 2.0 * graphWidth);

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            g.drawLine(plotpt, graphy, plotpt, graphy + 100);


            int bestGF = closest.getStatIndex(ex, ey);
            int bestGFPlotPoint1 = graphx + (int) (bestGF * (graphWidth / GF_ONE));

            g.setColor(new Color(1.0f, 0.9137255f, 0.14509805f, 0.8f));
            g.drawLine(bestGFPlotPoint1, graphy, bestGFPlotPoint1, graphy + 100);

            double guessfactor = (double) (lowestGFIndex - GF_ZERO) / (double) GF_ZERO;
            double angleOffset = direction * newWave.maxEscapeAngle / guessfactor;

            int bestGFPlotPoint2 = graphx + (int) (((double) closest.fireIndex / GF_ONE) * graphWidth);

            g.setColor(new Color(246, 15, 136, (int) (0.8 * 255)));
            g.drawLine(bestGFPlotPoint2, graphy, bestGFPlotPoint2, graphy + 100);

            g.setColor(new Color(196, 175, 246, (int) (0.8 * 255)));
            g.setFont(new Font("Verdana", Font.PLAIN, 10));
            g.drawString("Chosen GF", bestGFPlotPoint2 - 20, graphy - 10);
            g.drawString("Correct GF", bestGFPlotPoint1 - 25, graphy - 20);
            g.drawString("Closest Wave GF Plot", graphx, graphy + 105);

            for (int i = 0; i < closest.returnSegment.length; i++) {
                g.setColor(Color.MAGENTA);
                g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 1, graphy + (int) (closest.returnSegment[i] * 10), 2, 2);

                g.setColor(Color.GREEN);
                if (i == bestDensityIndex)
                    g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 2, graphy + (int) (closest.returnSegment[i] * 10), 4, 4);

                g.setColor(Color.CYAN);
                if (i == lowestDensityIndex)
                    g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 2, graphy + (int) (closest.returnSegment[i] * 10), 4, 4);

                cnt++;
            }
        }

        for (int i = 0; i < waves.size(); i++) {
            WaveBullet w = (WaveBullet) (waves.get(i));
            Point2D.Double center = new Point2D.Double(w.startX, w.startY);

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);

            double angleDivision = (w.maxEscapeAngle * 2 / w.returnSegment.length);
            int radius = (int) (w.getBulletSpeed() * (_robot.getTime() - w.fireTime));


            //Point2D.Double center = w.fireLocation;
            g.setColor(java.awt.Color.red);
            if (radius - 40 < center.distance(_radarScanner.nme.location))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            radius += w.getBulletSpeed();
            g.setColor(new Color(1, 0, 1, 0.5f));
            if (radius - 40 < center.distance(_radarScanner.nme.location))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            g.setColor(java.awt.Color.green);
            int cTime = (int) _robot.getTime();


            for (int p = 0; p < w.returnSegment.length; p++) {
                double angleOffset = angleDivision * (double) p * 0;

                float shade = (float) w.returnSegment[p];
                shade = (float) CTUtils.clamp(shade * 10, 0.0, 1.0);
                g.setColor(new Color(0, shade, 1, 1.0f));

                //System.out.print(shade + " ");
                //System.out.println("DA: " + w.directAngle + ", AD: " + angleDivision + ", MEA: " + w.maxEscapeAngle);
                Point2D.Double p2 = CTUtils.project(center, (w.directAngle + Math.PI) + (angleDivision * p) - (angleDivision * (w.returnSegment.length / 2)), (int) ((cTime - w.fireTime) * w.getBulletSpeed()));
                Point2D.Double p3 = CTUtils.project(p2, (w.directAngle + Math.PI) + (angleDivision * p) - (angleDivision * (w.returnSegment.length / 2)), (int) (w.getBulletSpeed()));

//                g.drawOval((int) (p2.x - 1), (int) (p2.y - 1), 2, 2);
                g.drawLine((int) p2.getX(), (int) p2.getY(), (int) p3.getX(), (int) p3.getY());
            }
        }


    }

    public double projectBearing(Situation s, double x, double y, double enemyHeading) {
        if (s == null)
            return 0;

        double[] currentStats = getStats(s); //stats[(int)(s.Distance / 100)];
        double absBearing = enemyHeading; //_robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // don't try to figure out the direction they're moving
        // they're not moving, just use the direction we had before
        if (_radarScanner.nme.velocity != 0) {
            if (Math.sin(_radarScanner.nme.headingRadians - absBearing) * _radarScanner.nme.velocity < 0)
                direction = -1;
            else
                direction = 1;
        }
        //w.bearingDirection = (double)direction *Math.asin(8D/newWave.getBulletSpeed())/GF_ZERO;

        newWave = new WaveBullet(x, y, absBearing, ((Double) _radarScanner._surfAbsBearings.get(0)).doubleValue(), _radarScanner.FIRE_POWER,
                direction, _robot.getTime(), currentStats);

        int bestGF = GF_ZERO;    // initialize it to be in the middle, guessfactor 0.
        for (int i = GF_ONE; i >= 0; i--)
            if (currentStats[i] > currentStats[bestGF])
                bestGF = i;

        newWave.fireIndex = bestGF;
        newWave.moves = getEnemyMoves();

        double lowBearing = CTUtils.absoluteBearing(new Point2D.Double(x, y), newWave.moves[0].location);
        double highBearing = CTUtils.absoluteBearing(new Point2D.Double(x, y), newWave.moves[newWave.moves.length - 1].location);

        int lowGF = newWave.getStatIndex(newWave.moves[0].location.getX(), newWave.moves[0].location.getY());
        int highGF = newWave.getStatIndex(newWave.moves[newWave.moves.length - 1].location.getX(), newWave.moves[newWave.moves.length - 1].location.getY());

        newWave.lowGF = (double) (lowGF - GF_ZERO) / (double) GF_ZERO;
        newWave.highGF = (double) (highGF - GF_ZERO) / (double) GF_ZERO;

        if (newWave.lowGF > newWave.highGF) {
            double copy = newWave.lowGF;
            newWave.lowGF = newWave.highGF;
            newWave.highGF = copy;
        }

        System.out.println("lowGF: " + newWave.lowGF + " highGF: " + newWave.highGF);

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double) (bestGF - GF_ZERO) / (double) GF_ZERO;

        /*
        if (_robot.getTime()/5 % 5 == 0)
        {
            guessfactor = (guessfactor+newWave.highGF)/2;
        }
        if (_robot.getTime()/5 % 5 == 1)
        {
            guessfactor = (guessfactor+newWave.lowGF)/2;
        }
        */


        //double angleOffset = CTUtils.clamp(direction * guessfactor, newWave.lowGF, newWave.highGF) * newWave.maxEscapeAngle;


        double angleOffset = direction < 0 ? -(guessfactor * newWave.lowGF * newWave.maxEscapeAngle) :
                (guessfactor * newWave.highGF * newWave.maxEscapeAngle);

//                CTUtils.clamp(direction * guessfactor, newWave.lowGF, newWave.highGF) * newWave.maxEscapeAngle;

        return angleOffset;
    }
}