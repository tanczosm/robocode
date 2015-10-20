package cg;

import robocode.*;
import robocode.control.events.BattleStartedEvent;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.*;     // for Point2D's
import java.io.IOException;
import java.util.ArrayList; // for collection of waves



import java.awt.event.KeyEvent;

import static java.awt.event.KeyEvent.*;

// TODO:
/*
 * Work on creating a queue of Scan objects.   Anytime a bullet is fired add the situational
 * Scan to the queue.   When an onBulletHit event or onBulletHitBullet event is triggered
 * call registerHit on the bullet in the queue who is closest to the actual bullet.   This will register
 * the outcome of that situation.
 * 
 */
public class CTSurferMove extends BaseMove {

    // Top borrowed from Moebius - Not used unless enemy isn't doing anything
    // Constants
    private static int historyIndex = 0;

    // End Moebius code
    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    public static boolean PaintPossibleSituations = false;

    public static double FIRE_POWER = 2;
    public static double FIRE_SPEED = 20 - FIRE_POWER * 3;

    public double tempRotate = 0;
    public double enemyProjectedDistance = 0;



    static final int GF_ZERO = 23; //23;
    static final int GF_ONE = 46; // 46;
    public static int BINS = GF_ONE + 1;
    static final double SCALING_FACTOR = 2;

    private static double _surfStats[][][] = new double[6][5][BINS];

    public static int lastDirection = 0;
    public static int lastVelocityChangeTime = -1;

    public boolean fireReady = true;
    public int consecutiveNonFiringWaves = 0;


    public ArrayList<EnemyWave> _enemyWaves;
    public static ArrayList<Point2D.Double> _hitLocations = new ArrayList<Point2D.Double>();

    public int getAway = 1;
    public int idealDistance = 450;


    // This is a rectangle that represents an 800x600 battle field,
    // used for a simple, iterative WallSmoothing method (by Kawigi).
    // If you're not familiar with WallSmoothing, the wall stick indicates
    // the amount of space we try to always have on either end of the tank
    // (extending straight out the front or back) before touching a wall.

    public static double WALL_STICK = 160;

    /*
    ArrayList<CrushTurtle.GunWave> gunWaves = new ArrayList<CrushTurtle.GunWave>();
    static double gunAngles[] = new double[16];

    static RaikoGun raikoGun; // For testing
    static final boolean waveTestMode = false;
    static final boolean gunTestMode = false;
    */
    public CTSurferMove (AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;

        _enemyWaves = new ArrayList<EnemyWave>();
    }

    public String getName()
    {
        return "CTSurfer";
    }

    public void scan(ScannedRobotEvent e) {

        double wallDistanceX = Math.min(_robot.getX() - 18, _radarScanner._fieldRect.getWidth() - _robot.getX() - 18);
        double wallDistanceY = Math.min(_robot.getY() - 18, _radarScanner._fieldRect.getHeight() - _robot.getY() - 18);
        double cornerDistance = CTUtils.cornerDistance(new Point2D.Double(_robot.getX(),_robot.getY()), _robot.getBattleFieldWidth(), _robot.getBattleFieldHeight());

        int wd = 0;
        if (cornerDistance < 100)
            wd = 1;
        else if (Math.min(wallDistanceX, wallDistanceY) < 60)
            wd = 2;

        //System.out.println("cd: " + cornerDistance + ", wdx: " + wallDistanceX + ", wdy: " + wallDistanceY);

        System.out.println("wd: " + wd);

        try {
            double bulletPower = _radarScanner._oppEnergy - e.getEnergy();
            if (bulletPower < 3.01 && (bulletPower > 0.09) && _radarScanner._surfDirections.size() > 2) {
                EnemyWave ew = new EnemyWave();
                ew.fireTime = _robot.getTime() - 1;
                ew.bulletPower = bulletPower;
                ew.bulletVelocity = CTUtils.bulletVelocity(bulletPower);
                ew.distanceTraveled = CTUtils.bulletVelocity(bulletPower);
                ew.direction = ((Integer) _radarScanner._surfDirections.get(2)).intValue();
                ew.directAngle = ((Double) _radarScanner._surfAbsBearings.get(2)).doubleValue();
                ew.fireLocation = (Point2D.Double) (_radarScanner.nme.lastlocation == null ? _radarScanner.nme.location.clone() : _radarScanner.nme.lastlocation.clone()); //_radarScanner.nme.location.clone(); // last tick
                ew.playerDistance = _radarScanner.nme.location.distance(_radarScanner._myLocation);
                ew.maxEscapeAngle = CTUtils.maxEscapeAngle(ew.bulletVelocity);
                ew.waveGuessFactors = _surfStats[(int) Math.min((_radarScanner._lastScan.getDistance() + 50) / 200, 3)][(int) (CTUtils.clamp((Math.abs(_radarScanner._lastLatVel) + 1) / 2, 0, 4))];
                _enemyWaves.add(ew);
                //double wf = CTUtils.wallDistance(ew.fireLocation.x, ew.fireLocation.y, ew.playerDistance, -ew.directAngle ,1);

//(int)((s.WallTriesBack+s.WallTriesForward)) (0..2)
                _radarScanner.nme.lastBulletPower = bulletPower;
            }


            if (bulletPower >= 0 && bulletPower < 3.01)
                _radarScanner.nme.lastShotTime = _robot.getTime();

        } catch (ArrayIndexOutOfBoundsException em) {
            //System.out.println(em.getMessage());
            System.out.println();
        }
    }

    public void update(ScannedRobotEvent e) {

        updateWaves();
        doSurfing();
    }




    public void updateWaves() {
        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

            ew.distanceTraveled = (_robot.getTime() - ew.fireTime) * ew.bulletVelocity;
            if (ew.distanceTraveled >
                    _radarScanner._myLocation.distance(ew.fireLocation) + 50) {
                //logHit(ew, _radarScanner._myLocation, 0.5);

                //int gfIndex = getFactorIndex(ew, _radarScanner._myLocation);

                _enemyWaves.remove(x);
                _radarScanner.nme.enemyShotMisses++;
                x--;
            }
        }
    }

    public static class BestWaves
    {
        public EnemyWave firstWave; // by time
        public EnemyWave secondWave; // by distance

        public BestWaves (EnemyWave first, EnemyWave second)
        {
            firstWave = first;
            secondWave = second;
        }
    }

    public ArrayList<EnemyWave> getBestWaves() {

        double bestTime = 50000;
        EnemyWave surfWave = null;
        ArrayList<EnemyWave> waves = new ArrayList<EnemyWave>();

        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);
            double distance = _radarScanner._myLocation.distance(ew.fireLocation) - ew.distanceTraveled;

            // figure out time to hit
            // d = vt, so t = d/v
            // Figure this out ...

            double t = distance / ew.bulletVelocity;
            if (t < bestTime && distance > ew.bulletVelocity)
            {
                waves.add(0, ew);
                bestTime = t;
            }
            /*

            if (distance > ew.bulletVelocity && distance < closestDistance) {
                surfWave = ew;
                closestDistance = distance;
            }
            */
        }
        //System.out.println(surfWave == null && _enemyWaves.size() > 0 ? "ERROR" : "");
        // return secondBestWave == null? surfWave : secondBestWave;
        return waves;
    }

    public BestWaves getBestSurfableWaves() {
        double closestDistance = 50000; // I juse use some very big number here
        double bestTime = 50000;
        EnemyWave secondBestWave = null;
        EnemyWave surfWave = null;

        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);
            double distance = _radarScanner._myLocation.distance(ew.fireLocation)
                    - ew.distanceTraveled;

            // figure out time to hit
            // d = vt, so t = d/v
            // Figure this out ...

            double t = distance / ew.bulletVelocity;
            if (t < bestTime && distance > ew.bulletVelocity)
            {
                secondBestWave = surfWave;
                surfWave = ew;
                bestTime = t;
            }
            /*

            if (distance > ew.bulletVelocity && distance < closestDistance) {
                surfWave = ew;
                closestDistance = distance;
            }
            */
        }
        //System.out.println(surfWave == null && _enemyWaves.size() > 0 ? "ERROR" : "");
        // return secondBestWave == null? surfWave : secondBestWave;
        return new BestWaves(surfWave, secondBestWave);
    }

    public static double targetSelection = 0;
    public static int targetHitEvents = 0;

    public static double targetMaxDanger = 0;

    public static void logHit(EnemyWave w, Point2D.Double targetLocation, double rollingDepth) {

        int gfIndex = getFactorIndex(w, targetLocation);
        int safestIndex = 0; // w.waveGuessFactors[gfIndex];
        int highestIndex = 0;

        for (int x = GF_ONE; x >= 0; x--)
        {
            if (w.waveGuessFactors[x] < w.waveGuessFactors[safestIndex])
            {
                safestIndex = x;
            }
            if (w.waveGuessFactors[x] > w.waveGuessFactors[highestIndex])
            {
                highestIndex = x;
            }
        }
        targetSelection += (w.waveGuessFactors[gfIndex] - w.waveGuessFactors[safestIndex]);
        targetMaxDanger += w.waveGuessFactors[highestIndex];
        targetHitEvents++;

        for (int x = GF_ONE; x >= 0; x--) {
            w.waveGuessFactors[x] = ((w.waveGuessFactors[x] * rollingDepth)
                    + ((1 + w.weight) / (Math.pow(x - gfIndex, SCALING_FACTOR) + 1)))
                    / (rollingDepth + 1 + w.weight);
        }


    }

    private static int getFactorIndex(EnemyWave w, Point2D.Double botLocation) {
        return (int) CTUtils.limit(0,
                ((((Utils.normalRelativeAngle(
                        w.absoluteBearing(botLocation)
                                - w.directAngle) * w.direction)
                        / Math.asin(8.0 / w.bulletVelocity))
                        * (GF_ZERO)) + (GF_ZERO)), GF_ONE);
    }

    public void onBulletHit(BulletHitEvent e) {
        _radarScanner._oppEnergy -= e.getEnergy();
        Situation best = _radarScanner.registerBulletHit(e.getBullet().getX(), e.getBullet().getY());

    }

    public void onBulletMissed(BulletMissedEvent e) {

        Situation best = _radarScanner.getInterceptingSituation(e.getBullet().getX(), e.getBullet().getY());

    }

    public void onBulletHitBullet(BulletHitBulletEvent e) {
        Bullet b = e.getBullet();
        _radarScanner.registerBulletHit(b.getX(), b.getY());

        //nme.enemyShotMisses++;  // When this bullet wave passes
        //_shotMisses++;

        logAndRemoveWave(new Point2D.Double(b.getX(),
                b.getY()));
    }

    public void onHitByBullet(HitByBulletEvent e) {

        _radarScanner.nme.enemyShotHits++;

        _hitLocations.add(new Point2D.Double(_robot.getX(),_robot.getY()));

        logAndRemoveWave(_radarScanner._myLocation);
    }

    public void logAndRemoveWave(Point2D.Double hitLocation) {

        if (!_enemyWaves.isEmpty()) {

            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++) {
                EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

                if (Math.abs(ew.distanceToPoint(hitLocation) - ew.distanceTraveled) < 100) {
                    hitWave = ew;
                    break;
                }
				/*
                if (Math.abs(ew.distanceTraveled -
                    hitLocation.distance(ew.fireLocation)) < 50
                    && Math.abs(bulletVelocity(e.getBullet().getPower()) 
                        - ew.bulletVelocity) < 0.001) {
                    hitWave = ew;
                    break;
                }*/
            }

            if (hitWave != null) {
                logHit(hitWave, hitLocation, 0.85);

                // We can remove this wave now, of course.
                _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
        }

    }


    // http://robowiki.net/wiki/Komarious/Code
    public static class futurePosition {
        public Point2D.Double predictedPosition;
        public double predictedHeading;
        public double predictedDistance;

        public futurePosition(Point2D.Double pPos, double ph, double pd) {
            this.predictedPosition = pPos;
            this.predictedHeading = ph;
            this.predictedDistance = pd;
        }
    }

    // CREDIT: mini sized predictor from Apollon, by rozu
    // http://robowiki.net?Apollon
    public futurePosition predictPosition(EnemyWave surfWave, int direction) {
        Point2D.Double predictedPosition = (Point2D.Double) _radarScanner._myLocation.clone();
        double predictedVelocity = _robot.getVelocity();
        double predictedHeading = _robot.getHeadingRadians();
        double maxTurning, moveAngle, moveDir, lastPredictedDistance;

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;


        // - actual distance traveled so far needs +bullet_velocity,
        //    because it's detected one after being fired
        // - another +bullet_velocity b/c bullet advances before collisions
        // ...So start the counter at 2.
        // - another +bullet_velocity to approximate a bot's half width
        // ...So start the counter at 3.
        counter = 3;
        do {
            moveDir = 1;

            if (Math.cos(moveAngle =
                    wallSmoothing(predictedPosition, surfWave.absoluteBearing(
                            predictedPosition) + (direction * 1.25 * getAway), direction)
                            - predictedHeading) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            // rozu comment:
            // this one is nice ;). if predictedVelocity and moveDir have different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
//            predictedVelocity += (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
//            predictedVelocity = limit(-8,
//                predictedVelocity + (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir),
//                8);

            // rozu comment:
            // calculate the new predicted position
            predictedPosition = CTUtils.project(predictedPosition,
                    predictedHeading = Utils.normalRelativeAngle(predictedHeading +
                            CTUtils.limit(-(maxTurning = Rules.getTurnRateRadians(Math.abs(predictedVelocity))),
                                    Utils.normalRelativeAngle(moveAngle), maxTurning)),
                    (predictedVelocity = CTUtils.limit(-8,
                            predictedVelocity + (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir),
                            8)));


        } while (
                (lastPredictedDistance = surfWave.distanceToPoint(predictedPosition)) >=
                        surfWave.distanceTraveled + ((++counter) * surfWave.bulletVelocity));

        return new futurePosition(predictedPosition, predictedHeading, lastPredictedDistance);
    }

    public double checkDanger2(EnemyWave surfWave, int direction) {

        int index = 0;
        futurePosition pos = predictPosition(surfWave, direction);
        double danger = (surfWave.waveGuessFactors[index = getFactorIndex(surfWave, pos.predictedPosition)]
                + .01 / (Math.abs(index - GF_ZERO) + 1))
                / Math.pow(pos.predictedDistance, 4);

        return danger;
    }
/*
    public double checkDangerOLD(EnemyWave surfWave, int direction) {

        int index = 0;
        double danger = 0;

        //double moveDirection;
        //double distanceRemaining = getDistanceRemaining();
        //if (distanceRemaining == 0) moveDirection = 0; else if (distanceRemaining < 0.0) moveDirection = -1; else moveDirection = 1;

        MovSimStat[] next = calculateFutureDanger (surfWave, direction, _radarScanner._myLocation.getX(), _radarScanner._myLocation.getY(),
                8,  getHeadingRadians(), getDistanceRemaining(), getTurnRemainingRadians());

        Graphics2D g = getGraphics();
        g.setColor(Color.MAGENTA);
        for (int i = 0; i < next.length; i++) {

            Point2D.Double predictedPosition = new Point2D.Double(next[i].x, next[i].y);
            double predictedDistance = surfWave.distanceToPoint(predictedPosition);

            int shade = (int)Math.min(255, next[i].danger*(255.0/3.0));

            g.setColor(new Color(shade, 255-shade, 0));
            g.drawOval((int) (predictedPosition.getX() - 1), (int) (predictedPosition.getY() - 1), 2, 2);

            System.out.println((direction < 0 ? "L " : "R ") + "P.dist: " + predictedDistance + ", Ticks until intercept: " + next[i].tickDistance + ", Danger: " + next[i].danger);

            // future danger at any given predicted position is Math.min(dangerLeft, dangerRight)

            danger += next[i].danger * (1.0/(i+1));
        }

        return danger;
    }
*/

    public double checkDanger(ArrayList<EnemyWave> waves, int direction) {

        int index = 0;
        double danger = 0;

        if (waves.size() == 0)
            return 0;

        EnemyWave surfWave = waves.get(0);

        //double moveDirection;
        //double distanceRemaining = getDistanceRemaining();
        //if (distanceRemaining == 0) moveDirection = 0; else if (distanceRemaining < 0.0) moveDirection = -1; else moveDirection = 1;
        double absBearing = CTUtils.absoluteBearing(surfWave.fireLocation, _radarScanner._myLocation);

        RobotState[] next = calculateFutureDanger (waves, _radarScanner._myLocation.getX(), _radarScanner._myLocation.getY(),
                absBearing,
                _robot.getVelocity(),  8, _robot.getHeadingRadians(), 0, _robot.getDistanceRemaining(), direction == -1 ? true : false, _robot.getTime(), _radarScanner._fieldRect, 800, 600, 140, false );

        int bestIndex = findSafestPoint(next, surfWave, false);
        RobotState best = (bestIndex == -1 ? null : next[bestIndex]);

        Graphics2D g = _robot.getGraphics();
        g.setColor(Color.MAGENTA);
        for (int i = 0; i < next.length; i++) {

            Point2D.Double predictedPosition = next[i].location; //new Point2D.Double(next[i].location.getX(), next[i].location.getY());
            double predictedDistance = surfWave.distanceToPoint(predictedPosition);

            int shade = (int)Math.min(255, next[i].danger*(255.0/3.0));

            g.setColor(new Color(shade, 255-shade, 0));
            g.drawOval((int) (predictedPosition.getX() - 1), (int) (predictedPosition.getY() - 1), 2, 2);

            //System.out.println((direction < 0 ? "L " : "R ") + "P.dist: " + predictedDistance + ", Ticks until intercept: " + next[i].tickDistance + ", Danger: " + next[i].danger);

            // future danger at any given predicted position is Math.min(dangerLeft, dangerRight)

            danger += next[i].danger; // * (1.0/(i+1));
        }

        if (best != null)
        {
            g.setColor(Color.CYAN);
            g.drawOval((int) (best.location.getX() - 3), (int) (best.location.getY() - 3), 6, 6);
            g.drawString(best.dangerDensity + "", (float)best.location.getX(), (float)best.location.getY()+10);
            //return best.dangerDensity;
        }

        return danger;
    }


    public int findSafestMove(ArrayList<EnemyWave> waves, double attackAngle) {

        int middleIndex = 0;
        int index = 0;
        double danger = 0;
        double dangerSum = 0, dangerMean, dangerStandardDev, sDevCalc = 0;

        if (waves.size() == 0)
            return 1;

        EnemyWave firstWave = waves.get(0);

        //double moveDirection;
        //double distanceRemaining = getDistanceRemaining();
        //if (distanceRemaining == 0) moveDirection = 0; else if (distanceRemaining < 0.0) moveDirection = -1; else moveDirection = 1;
        double absBearing = CTUtils.absoluteBearing(firstWave.fireLocation, _radarScanner._myLocation);

        RobotState[] left = calculateFutureDanger (waves, _radarScanner._myLocation.getX(), _radarScanner._myLocation.getY(),
                absBearing,
                _robot.getVelocity(),  8, _robot.getHeadingRadians(), attackAngle, _robot.getDistanceRemaining(), true, _robot.getTime(), _radarScanner._fieldRect, 800, 600, 140, false );

        RobotState[] right = calculateFutureDanger (waves, _radarScanner._myLocation.getX(), _radarScanner._myLocation.getY(),
                absBearing,
                _robot.getVelocity(),  8, _robot.getHeadingRadians(), attackAngle, _robot.getDistanceRemaining(), false, _robot.getTime(), _radarScanner._fieldRect, 800, 600, 140, false );

        RobotState[] next = new RobotState[left.length+right.length];
        middleIndex = left.length;

        int j = 0;
        for (int i = left.length-1; i >= 0; i--) {
            next[j++] = left[i];
        }

        for (int i = 0; i < right.length; i++) {
            next[i + middleIndex] = right[i];

        }


        int bestIndex = findSafestPoint(next, firstWave, false);
        int safestIndex = findSafestPoint(next, firstWave, true);

        for (int i = 0; i < next.length; i++)
        {
            dangerSum += next[i].dangerDensity;
        }
        dangerMean = dangerSum / (double)next.length;

        // EXPERIMENTAL:
        /*
        if (next[0].location.distance(_radarScanner.nme.location) < next[next.length-1].location.distance(_radarScanner.nme.location))
        {
            if (bestIndex < middleIndex)
                if (safestIndex > middleIndex)
                    bestIndex = safestIndex;
                else
                    bestIndex = middleIndex;
        }*/

        RobotState best = (bestIndex == -1 ? null : next[bestIndex]);
        RobotState safest = (safestIndex == -1 ? null : next[safestIndex]);

        Graphics2D g = _robot.getGraphics();
        double graphWidth = 300;
        int graphx = (int)_robot.getBattleFieldWidth()-((int)graphWidth+30), graphy=100, cnt=0;

        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy, graphx+((int)graphWidth), graphy);
        g.drawLine(graphx, graphy, graphx, graphy+100);
        g.drawLine(graphx+((int)graphWidth), graphy, graphx+(int)graphWidth, graphy+100);

        g.setColor(Color.MAGENTA);


        for (int i = 0; i < next.length; i++) {

            sDevCalc += Math.pow(next[i].dangerDensity - dangerMean, 2);

            Point2D.Double predictedPosition = next[i].location; //new Point2D.Double(next[i].location.getX(), next[i].location.getY());
            //double predictedDistance = bestWaves.firstWave.distanceToPoint(predictedPosition);

            int shade = (int)Math.min(255, next[i].danger*(255.0/5.0));

            g.setColor(new Color(shade, 255-shade, 0));
            g.drawOval((int) (predictedPosition.getX() - 1), (int) (predictedPosition.getY() - 1), 2, 2);

            g.setColor(Color.YELLOW);
            if (Math.abs(middleIndex-i) < next[i].tickDistance)
                g.drawOval((int) (predictedPosition.getX() - 2), (int) (predictedPosition.getY() - 2), 4, 4);

            //System.out.println("P.dist: " + predictedDistance + ", Ticks until intercept: " + next[i].tickDistance + ", Danger: " + next[i].danger);

            // future danger at any given predicted position is Math.min(dangerLeft, dangerRight)
            if (i == bestIndex || i == safestIndex)
                g.setColor(Color.CYAN);
            g.drawOval(graphx+(int)(cnt*(graphWidth/next.length)), graphy+(int)(next[i].dangerDensity), 2, 2);
            cnt++;


            danger += next[i].danger; // * (1.0/(i+1));
        }
        dangerStandardDev = Math.sqrt(sDevCalc / (next.length));

        g.setColor(Color.BLUE);
        g.drawLine(graphx, graphy+(int)dangerMean, graphx+(int)graphWidth, graphy+(int)dangerMean);

        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy+(int)(dangerMean-1*dangerStandardDev), graphx+(int)graphWidth, graphy+(int)(dangerMean-1*dangerStandardDev));

        g.setColor(Color.YELLOW);
        g.drawLine(graphx, graphy+(int)(dangerMean-2*dangerStandardDev), graphx+(int)graphWidth, graphy+(int)(dangerMean-2*dangerStandardDev));

        g.setColor(Color.ORANGE);
        g.drawLine(graphx, graphy+(int)(dangerMean-3*dangerStandardDev), graphx+(int)graphWidth, graphy+(int)(dangerMean-3*dangerStandardDev));

        java.text.DecimalFormat df = new java.text.DecimalFormat("#.##");
        if (best != null)
        {
            g.setColor(Color.CYAN);
            g.drawOval((int) (best.location.getX() - 3), (int) (best.location.getY() - 3), 6, 6);
            g.drawString(df.format(best.danger) + "", (float)best.location.getX(), (float)best.location.getY()+10);

            //return best.dangerDensity;
        }

        int movement = (bestIndex < middleIndex ? -1 : (bestIndex > middleIndex ? 1 : 0));

        if (safest != null) {
            double checkDist = Math.abs(best.location.distance(safest.location));
            double safestDist = Math.abs(next[middleIndex].location.distance(safest.location));

            //return best.dangerDensity;

            int possibleTicks = Math.abs(middleIndex-safestIndex);

            // avoid the "safest" spot as well
            if (possibleTicks < safest.tickDistance && checkDist > 70) {
                g.setColor(Color.GREEN);
/*
                movement = (safestIndex < middleIndex ? -1 : (safestIndex > middleIndex ? 1 : 0));

                if (safestDist < 32)
                    movement = 0;
*/

            }
            else {
                g.setColor(Color.RED);
            }

            g.drawOval((int) (safest.location.getX() - 3), (int) (safest.location.getY() - 3), 6, 6);
            g.drawString(df.format(safest.danger) + "", (float)safest.location.getX(), (float)safest.location.getY()+10);

            g.drawString(safest.tickDistance + " " + possibleTicks + " Distance: " + df.format(checkDist) + "", (float)next[(safestIndex+bestIndex)/2].location.getX(), (float)next[(safestIndex+bestIndex)/2].location.getY()+10);  // (float)(safest.location.getX()+best.location.getX())/2f, (float)(safest.location.getY()+best.location.getY())/2f+10);

            g.setColor(Color.PINK);
            g.drawString("Mean: " + dangerSum + " Danger: " + df.format(25-next[middleIndex].dangerDensity), (float)_robot.getBattleFieldHeight()-20f, 10f);

        }

        return movement;
    }

    /*
    public MovSimStat[] calculateFutureDanger (EnemyWave surfWave, int direction, double x, double y, double velocity, double heading, double distanceRemaining, double angleToTurn)
    {
        int index = 0;
        int possibleTicks = CTUtils.bulletTicks(surfWave.distanceToPoint(new Point2D.Double(x,y)) - surfWave.distanceTraveled ,surfWave.bulletPower) - 2;

        MovSim mv = CTUtils.getMovSim();

        MovSimStat[] next = mv.futurePos((int)Math.max(1, possibleTicks), x, y,
                velocity*direction,  8, heading, Math.abs(distanceRemaining)*direction, angleToTurn,
                10.0, 800d, 600d);

        for (int i = 0; i < next.length; i++) {

            Point2D.Double predictedPosition = new Point2D.Double(next[i].x, next[i].y);
            double predictedDistance = surfWave.distanceToPoint(predictedPosition);
            int ticks = CTUtils.bulletTicks(predictedDistance - surfWave.distanceTraveled ,surfWave.bulletPower) - 2;

            double dg = (surfWave.waveGuessFactors[index = getFactorIndex(surfWave, predictedPosition)]
                    + .01 / (Math.abs(index - GF_ZERO) + 1))
                    / Math.pow(predictedDistance, 4);

            double dgs = Math.log10(dg*1000000000000000d);

            next[i].danger = dgs;
            next[i].tickDistance = ticks;
            next[i].direction = direction;
        }

        return next;
    }
     */

    public double calculateScaledDanger (EnemyWave surfWave, Point2D.Double location)
    {
        int index = 0;

        double predictedDistance = surfWave.distanceToPoint(location);
        double dg = (surfWave.waveGuessFactors[index = getFactorIndex(surfWave, location)]
                + .01 / (Math.abs(index - GF_ZERO) + 1))
                / Math.pow(predictedDistance, 4);

        double dgs = Math.log10(dg*1000000000000000d);  // This is just 15*dg isn't it?

        return dgs;
    }

    public int findSafestPoint (RobotState[] points, EnemyWave surfWave, boolean safest)
    {
        double bestDensity = (safest ? 0 : Double.MAX_VALUE);
        int bestIndex = 0;

        if (points.length == 0)
            return -1;

        //surfWave.maxEscapeAngle
/*
    if (targetSelection/targetHitEvents < 0.1) {

    }
 */

        for (int i = points.length; --i >= 0; ) {

            double density = 0;
            double u;

            double flattener = (_robot.getRoundNum()/(double)_robot.getNumRounds()) * 5.0 +1;
            flattener = 1d; // No flattener

            for (int j = points.length; --j >= 0; ) {
                //double kdeDiff = points[i].danger - points[j].danger;
                double kdeDiff = points[j].danger - points[i].danger;
                //double kdeDiff = Math.sqrt(Math.pow(points[i].danger - points[j].danger,2));
                //double kdeDiff = Math.abs(bestNodes.get(i).Distance - bestNodes.get(j).Distance);

                density += Math.exp(
                        (
                                u = (kdeDiff * flattener) /// 2 // BAND_WIDTH OF 4 seems ok
                        )
                                * u
                                * -0.5d
                );
            }

            double absBearing = Math.abs(Utils.normalRelativeAngle(surfWave.absoluteBearing(points[i].location) - surfWave.directAngle));
//System.out.println("Bearing: " + absBearing + ", mea: " + surfWave.maxEscapeAngle + ", dA: " + surfWave.directAngle);
            points[i].dangerDensity = density;

            if (safest && density > bestDensity)
            {
                bestDensity = density;
                bestIndex = i;
            }
            else if (!safest && density < bestDensity /*&& absBearing <= surfWave.maxEscapeAngle*/) {

                bestDensity = density;
                bestIndex = i;

            }


        }

        return bestIndex;
    }

    // Calculate the path a robot can take with smoothing..
    public RobotState[] calculateFutureDanger (ArrayList<EnemyWave> waves, double x, double y, double absBearingRadians, double velocity, double maxVelocity, double heading, double attackAngle, double distanceRemaining, boolean clockwise, long currentTime, Rectangle2D.Double battleField,	double bfWidth, double bfHeight,
                                               double wallStick, boolean ignoreWallHits)
    {
        int index = 0;
        EnemyWave surfWave = waves.get(0);

        double waveDist = surfWave.distanceToPoint(new Point2D.Double(x,y)) - surfWave.distanceTraveled;
        waveDist = Math.min(_radarScanner.nme.location.distance(x,y), waveDist); // Take the smaller of the two distances

        int possibleTicks = CTUtils.bulletTicks(waveDist ,surfWave.bulletPower) - 2;
//        int movesToFind = (int)CTUtils.clamp(3 * possibleTicks, possibleTicks, 50); // Presently 3 and 50 are optimal
        int movesToFind = (int)CTUtils.clamp(3 * possibleTicks, 20, 50); // Presently 3 and 50 are optimal

        RobotState[] moves = new  RobotState[movesToFind];

        MovSim mv = CTUtils.getMovSim();



        double wallDanger = 0;
        Rectangle2D.Double field = new Rectangle2D.Double(_radarScanner._fieldRect.x+20,_radarScanner._fieldRect.y+20,
                _radarScanner._fieldRect.width-40,_radarScanner._fieldRect.height-40);

        for (int i = 0; i < movesToFind; i++) {

            RobotState next = CTUtils.nextPerpendicularWallSmoothedLocation(
                    new Point2D.Double(x,y), absBearingRadians,velocity, maxVelocity, heading,
                    attackAngle, clockwise, currentTime,
                    battleField, bfWidth, bfHeight,
                    wallStick, ignoreWallHits);


            Point2D.Double predictedPosition = next.location;
            double predictedDistance = surfWave.distanceToPoint(predictedPosition);
            int ticks = CTUtils.bulletTicks(predictedDistance - surfWave.distanceTraveled ,surfWave.bulletPower) - 2;

            double dgs = calculateScaledDanger (surfWave, predictedPosition);


            if (!field.contains(next.location))
                wallDanger = 0.1; // BEST IS = 0.1

            for (int z = 1; z < waves.size(); z++)
                dgs += calculateScaledDanger (waves.get(z), predictedPosition); // * (1.0/(z));


            /*
            dgs += 5*(1/_radarScanner.nme.distance);
            dgs += 5*(1- 1/Point2D.distance(predictedPosition.getX(),predictedPosition.getY(),_radarScanner._fieldRect.getCenterX(), _radarScanner._fieldRect.getCenterY()));
            */
            dgs *= (1+surfWave.bulletPower);
            //dgs += wallDanger;   // Need to somehow manage positioning better.. walls are death

            next.danger = dgs;
            next.tickDistance = ticks;
            //next.direction = direction;

            x = next.location.getX();
            y = next.location.getY();
            heading = next.heading;
            velocity = next.velocity;
            absBearingRadians = CTUtils.absoluteBearing(surfWave.fireLocation, next.location);

            moves[i] = next;
        }

        return moves;
    }

    public double cornerDistance (Point2D.Double location)
    {
        double tr = location.distance(_robot.getBattleFieldWidth(), _robot.getBattleFieldHeight());
        double tl = location.distance(0, _robot.getBattleFieldHeight());
        double br = location.distance(_robot.getBattleFieldWidth(), 0);
        double bl = location.distance(0, 0);

        return Math.min(Math.min(tr,tl), Math.min(br,bl));
    }

    public void doSurfing() {

        //BestWaves best = getBestSurfableWaves();
        ArrayList<EnemyWave> best = getBestWaves();
        //EnemyWave surfWave = best.firstWave;
		/*
		double distdiff = idealDistance - nme.distance;
		getAway = 0;
		if (distdiff < 100)
			getAway = -1;
		else if (distdiff > 100)
			getAway = 1;
*/

        if (best.size() == 0) {

            consecutiveNonFiringWaves++;

            if (consecutiveNonFiringWaves < 450)
                return;
            //System.out.println("Moving in");

            _robot.setMaxVelocity(1000);

            ScannedRobotEvent e = _radarScanner._lastScan;

            // Assign ArcMovement here to save a byte with the targetBearing assign.
            double arcMovement = e.getVelocity() * Math.sin(e.getHeadingRadians() - 0); //_ _targeting._lastBearing);


            // Move in a SHM oscollator pattern with a bit of random thrown in for good measure.
            double goAhead = Math.cos(historyIndex++ >> 4) * 150 + 50;
            _robot.setAhead(goAhead);


            double ang = e.getBearingRadians() + Math.PI / 2 - 0.08*CTUtils.sign(goAhead); // wallSmoothing(_radarScanner._myLocation, e.getBearingRadians() + Math.PI / 2,  CTUtils.sign(goAhead));

            // Try to stay equa-distance to the target -  a slight movement towards
            // target would help with corner death, but no room.
            _robot.setTurnRightRadians(ang);

            //setTurnGunRightRadians(Utils.normalAbsoluteAngle(e.getHeadingRadians() + _targeting._lastBearing - getGunHeadingRadians()));

            return;
        }
        consecutiveNonFiringWaves = 0;

        int index;



        /*
        double dangerMiddle = (surfWave.waveGuessFactors[index = getFactorIndex(surfWave, _radarScanner._myLocation)]
                + .01 / (Math.abs(index - GF_ZERO) + 1))
                / Math.pow(surfWave.distanceToPoint(_radarScanner._myLocation), 4);
        */
        double cornerDist = cornerDistance(_radarScanner._myLocation);
        double enemyCornerDist = cornerDistance(_radarScanner.nme.location);

        getAway = -1;

        int avoidCornerDistance = 150;
        if (cornerDist < avoidCornerDistance && !(_radarScanner.nme.distance < avoidCornerDistance && enemyCornerDist < cornerDist))
            getAway = 1; // * -CTUtils.sign(getDistanceRemaining());;

        if ((_radarScanner.nme.energy-_robot.getEnergy()) > 40)
            getAway = 1;


        double goAngle = CTUtils.absoluteBearing(best.get(0).fireLocation, _radarScanner._myLocation);
        double avoid = 0.36 * getAway;

        if (getAway == 1)
            avoid = 0.4;

        /*
        if (_radarScanner._myLocation.distance(_radarScanner.nme.location) < 250)
            avoid = -0.2;
        */

        /*
        else if (_radarScanner._myLocation.distance(_radarScanner.nme.location) > 500)
            avoid = -0.1;
        */

        /*
        // THIS WORKS REASONABLY WELL
        double dangerLeft = 25-checkDanger(best.firstWave, best.secondWave, -1);
        double dangerRight = 25-checkDanger(best.firstWave, best.secondWave, 1);
        */

//avoid = -0.5 * CTUtils.sign(getDistanceRemaining());
        int move = findSafestMove(best, avoid);

        /*
        if (lastDirection != move)
        {
            if (getTime()-lastVelocityChangeTime == 1)
            {
                move = lastDirection;
            }
            else
                lastVelocityChangeTime = (int)getTime();
        }

        lastDirection = move;
        */

        double dangerLeft = (move == -1 ? 0 : 1);
        double dangerRight = (move == -1 ? 1 : 0);
        double dangerMiddle = 5;

        if (move == 0)
        {
            dangerLeft = 1;
            dangerRight = 1;
            dangerMiddle = 0;
        }

		/* 
		if (dangerMiddle < dangerRight && dangerMiddle < dangerLeft)
		{
			setMaxVelocity(0);
		}
        else
		*/


        //java.text.DecimalFormat df = new java.text.DecimalFormat("#.##");
        //System.out.println("Danger [L: " + df.format(dangerLeft) + //*100000000000000d
        //                    /*" M: " + df.format(dangerMiddle*100000000000000d) +*/
        //                " R: " + df.format(dangerRight) + "] Corner Distance: [" + cornerDist + "]"
        //);


        if (dangerMiddle < dangerLeft && dangerMiddle < dangerRight)
        {
            System.out.println("Jamming on the brakes");
            _robot.setMaxVelocity(0);
            goAngle -= (Math.PI/2);
        }

        else if (dangerLeft < dangerRight) {

            _robot.setMaxVelocity(1000);
            goAngle = wallSmoothing(_radarScanner._myLocation, goAngle - (Math.PI / 2) - avoid, -1);
        } else {
            _robot.setMaxVelocity(1000);
            goAngle = wallSmoothing(_radarScanner._myLocation, goAngle + (Math.PI / 2) + avoid, 1);
        }

        CTUtils.setBackAsFront(_robot, goAngle);
    }


    // CREDIT: Iterative WallSmoothing by Kawigi
    //   - return absolute angle to move at after account for WallSmoothing
    // robowiki.net?WallSmoothing
    public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation) {
        while (!RadarScanner._fieldRect.contains(CTUtils.project(botLocation, angle, 160))) {
            angle += orientation * 0.05;
        }
        return angle;
    }


    public void onPaint(java.awt.Graphics2D g) {

        g.setColor(new Color(0.7882353f, 0.18039216f,1f));
        g.setFont(new Font("Verdana", Font.PLAIN, 12));
        g.drawString("Avg.Danger.Above.Min: " + targetSelection/(double)targetHitEvents, 5, 590);
        g.drawString("Avg.Danger.Max: " + targetMaxDanger/(double)targetHitEvents, 5, 576);

        for (Point2D.Double hit : _hitLocations)
        {
            g.setColor(Color.pink);
            g.drawOval((int) (hit.getX() - 2), (int) (hit.getY() - 2), 4, 4);

        }

        for (int i = 0; i < _enemyWaves.size(); i++) {
            EnemyWave w = (EnemyWave) (_enemyWaves.get(i));
            Point2D.Double center = w.fireLocation;

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);
            //hack to make waves line up visually, due to execution sequence in robocode engine
            //use this only if you advance waves in the event handlers (eg. in onScannedRobot())
            //NB! above hack is now only necessary for robocode versions before 1.4.2
            //otherwise use: 
            int radius = (int) w.distanceTraveled;

            //Point2D.Double center = w.fireLocation;
            g.setColor(java.awt.Color.red);
            if (radius - 40 < center.distance(_radarScanner._myLocation))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            radius += w.bulletVelocity;
            g.setColor(new Color(1, 0, 1, 0.5f));
            if (radius - 40 < center.distance(_radarScanner._myLocation))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            g.setColor(java.awt.Color.green);
            int cTime = (int)_robot.getTime();

            double angleDivision = (w.maxEscapeAngle*2 / w.waveGuessFactors.length);
            for (int p = 0; p < w.waveGuessFactors.length; p++)
            {
                double angleOffset = angleDivision*(double)p * 0;

                float shade = (float)w.waveGuessFactors[p];
                shade = (float)CTUtils.clamp(shade*10, 0.0, 1.0);
                g.setColor(new Color(0, shade, 1, 1.0f));

                //System.out.print(shade + " ");
                //System.out.println("DA: " + w.directAngle + ", AD: " + angleDivision + ", MEA: " + w.maxEscapeAngle);
                Point2D.Double p2 = CTUtils.project(w.fireLocation, w.directAngle + (angleDivision*p) - (angleDivision*(w.waveGuessFactors.length/2)), (int) ((cTime - w.fireTime) * w.bulletVelocity));
                Point2D.Double p3 = CTUtils.project(p2, w.directAngle + (angleDivision*p) - (angleDivision*(w.waveGuessFactors.length/2)), (int) (w.bulletVelocity));

//                g.drawOval((int) (p2.x - 1), (int) (p2.y - 1), 2, 2);
                g.drawLine((int)p2.getX(), (int)p2.getY(), (int)p3.getX(), (int)p3.getY());
            }
        }

        java.text.DecimalFormat df = new java.text.DecimalFormat("#.##");

        double bestDistance = 0;

        double _shotsTotal = 0; // playerStats.getOverallStat().getTotal();
        double _shotAccuracy = 0; // playerStats.getOverallStat().getHitRatio() * 100;

        double _enemyShotsTotal = _radarScanner.nme.enemyShotHits + _radarScanner.nme.enemyShotMisses;
        double _enemyShotAccuracy = (_enemyShotsTotal == 0 ? 0 : ((_radarScanner.nme.enemyShotHits / (_enemyShotsTotal)) * 100));

        double enemyBulletPower =   _radarScanner.nme.lastBulletPower;// Enemy.getLastBulletPower(FIRE_POWER);
		 
		 
 		 /*

		 
		 if (_surfAbsBearings.size() > 0)
		 {
			 double absBearing =  _surfAbsBearings.get(0) - Math.PI;
					
			 Point2D.Double p1 = project(_myLocation, absBearing+mea, 100);
			 Point2D.Double p2 = project(_myLocation, absBearing-mea, 100);
			
			 g.drawLine((int)_myLocation.getX(), (int)_myLocation.getY(), (int)p1.getX(), (int)p1.getY());
			 g.drawLine((int)_myLocation.getX(), (int)_myLocation.getY(), (int)p2.getX(), (int)p2.getY());
		 }
		 */

        g.setColor(java.awt.Color.white);
        g.setFont(new Font("Verdana", Font.PLAIN, 15));
        g.drawString("Firing Power: " + df.format(_radarScanner.FIRE_POWER) + " (Enemy Distance: " + df.format(_radarScanner.nme.distance) + "), Enemy FP: " + df.format(enemyBulletPower), 10, 20 + 18 * 2);
        g.drawString("Enemy Shot Accuracy: " + df.format(_enemyShotAccuracy) + "%", 10, 20 + 18);
        g.drawString("Shot Accuracy: " + df.format(_shotAccuracy) + "%", 10, 20);

        //g.drawString("KNN GUN [K:  " + MAX_SELECTED_SITUATIONS + ", BANDWIDTH: " + KERNEL_DENSITY_BANDWIDTH + "]", 250, 20);

        //g.drawString("Ideal Distance: " + df.format(bestDistance), 10, 20 + 18 + 18 + 18);

    }

    public void onKeyPressed(KeyEvent e) {
        switch (e.getKeyCode()) {
            case VK_UP:
            case VK_W:
                //KERNEL_DENSITY_BANDWIDTH++;
                break;

            case VK_DOWN:
            case VK_S:
                //KERNEL_DENSITY_BANDWIDTH--;
                break;

            case VK_P:
                PaintPossibleSituations = !PaintPossibleSituations;
                break;
        }
    }







}