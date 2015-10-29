package cg;

import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.basic.BasicMLDataPair;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.neural.networks.training.propagation.back.Backpropagation;
import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

/**
 * Created by tanczosm on 10/14/2015.
 *
 * IMPROVEMENT IDEAS:
 * - Add distancing capabilities to try and maintain a distance of roughly 450 pixels whenever possible
 * - Instead of always slowing down to reach the end point, try altering the path so it goes a different way
 *
 */
public class GTSurferMove extends BaseMove {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    private boolean showDebugGraphics = true;

    // Neural network stuff
    public BasicNetwork basicNetwork;
    public Backpropagation basicTrain;
    private ArrayList<MLDataPair> _theData;
    private ArrayList<MLDataPair> _lastHitsData;

    public static final int INPUT_LENGTH = 49;
    public static final int OUTPUT_LENGTH = 113;

    public static int BINS = OUTPUT_LENGTH;
    public static double _surfStats[] = new double[BINS]; // we'll use 47 bins
    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _enemyLocation;  // enemy bot's location

    public Point2D.Double _lastGoToPoint;
    public double _lastGunHeat;
    public double direction = 1;

    public ArrayList<EnemyWave> _enemyWaves;
    public ArrayList _surfData;
    public ArrayList<Double> _LateralVelocityLast10;
    private final LinkedList<Bullet> bullets = new LinkedList<Bullet>();

    // TEMPORARY
    int debugFactorIndex = 0;
    int debugCurrentIndex = 0;


    double _lastVelocity = 0;
    int _lastDirection = 1;
    int _lastDirectionTimeChange = 0;

    private class surfData
    {
        public Point2D.Double enemyLocation;
        public Point2D.Double playerLocation;

        public int direction;
        public double lateralVelocity;
        public double advancingVelocity;
        public double lateralDistanceLast10;
        public double absBearing;
        public double acceleration;
        public int timeSinceDirectionChange;
        public double forwardWallDistance;
        public double reverseWallDistance;
    }

    public GTSurferMove (AdvancedRobot robot, RadarScanner radarScanner)
    {
        _robot = robot;
        _radarScanner = radarScanner;

        _enemyWaves = new ArrayList<EnemyWave>();
        _surfData = new ArrayList();
        _LateralVelocityLast10 = new ArrayList<Double>();
        _enemyLocation = new Point2D.Double();
        _lastGunHeat = _robot.getGunHeat();

        basicNetwork = new BasicNetwork();
        basicNetwork.addLayer(new BasicLayer(null, true, INPUT_LENGTH));
        //basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 39));
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, OUTPUT_LENGTH));
        basicNetwork.getStructure().finalizeStructure();
        basicNetwork.reset();
        basicNetwork.reset(1000);

        _theData = new ArrayList<MLDataPair>();
        _lastHitsData = new ArrayList<MLDataPair>();
        _theData.add(new BasicMLDataPair(new BasicMLData(new double[INPUT_LENGTH]), new BasicMLData(new double[OUTPUT_LENGTH])));
        MLDataSet trainingSet = new BasicMLDataSet(_theData);
        basicTrain = new Backpropagation(basicNetwork, trainingSet, 0.7, 0.3);
        basicTrain.setBatchSize(1);

    }

    public String getName ()
    {
        return "GTSurfer";
    }

    /* Call mover events */
    public void onBulletHit(BulletHitEvent e) {
    }


    public void onBulletMissed(BulletMissedEvent e)
    {
    }

    private void calculateShadow(final EnemyWave wave, final Bullet b) {

        if (b == null)
            return;

        Graphics g = _robot.getGraphics();

        // until bullet is past wave calculate ahead
        long timeOffset = 0;
        final double x = b.getX();
        final double y = b.getY();
        final double h = b.getHeadingRadians();
        final double v = b.getVelocity();
        int colr = 0;
        do {
            final double r = wave.getRadius(_robot.getTime() + timeOffset);
            final Line line = Line.projection(x, y, h, v * timeOffset, v * (timeOffset + 1));
            //System.out.println("Calculating shadow..");
            if(_myLocation.distanceSq(line.x1, line.y1) > wave.fireLocation.distanceSq(_myLocation) - r * r) break;

            /*
            if (colr++ % 2 == 0)
                g.setColor(new Color(128, 180, 255));
            else
                g.setColor(new Color(70, 70, 255));

            g.drawLine((int)line.x1, (int)line.y1, (int)line.x2, (int)line.y2);
            */
            //g.setColor(new Color(255, 83, 70));
            //g.drawOval((int)wave.fireLocation.x-(int)(r/2), (int)wave.fireLocation.y-(int)(r/2),(int)r, (int)r);

            int sc = wave.bulletShadows.size();
            wave.shadowBullet(b, line, _robot.getTime() + timeOffset, g);
            //if (sc < wave.bulletShadows.size())
            //    wave.safePoints = null;
        }
        while(++timeOffset < 110);
    }

    private void calculateShadowsForBullet(final Bullet b) {
        for(final EnemyWave wave : _enemyWaves) {
            //if(!wave.isReal) continue;
            calculateShadow(wave, b);
        }
    }

    private void calculateShadowsForWave(final EnemyWave wave) {
        //if(wave.imaginary) return;
        final Iterator<Bullet> it = bullets.iterator();
        while(it.hasNext()) {
            final Bullet b = it.next();
            if(b == null || !b.isActive()) {
                it.remove();
                continue;
            }
            calculateShadow(wave, b);
        }
    }

    // This is a rectangle that represents an 800x600 battle field,
// used for a simple, iterative WallSmoothing method (by Kawigi).
// If you're not familiar with WallSmoothing, the wall stick indicates
// the amount of space we try to always have on either end of the tank
// (extending straight out the front or back) before touching a wall.
    public static Rectangle2D.Double _fieldRect
            = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public static double WALL_STICK = 160;

    public void scan(ScannedRobotEvent e) {
        _myLocation = new Point2D.Double(_robot.getX(), _robot.getY());

        double lateralVelocity = _robot.getVelocity()*Math.sin(e.getBearingRadians());
        double absBearing = e.getBearingRadians() + _robot.getHeadingRadians();
        double velocity = _robot.getVelocity();
        int direction = new Integer((lateralVelocity >= 0) ? 1 : -1);
        int timeSinceDirectionChange = 0;

        _robot.setTurnRadarRightRadians(Utils.normalRelativeAngle(absBearing - _robot.getRadarHeadingRadians()) * 2);

        double acceleration = 0;
        if (_lastVelocity != Double.MAX_VALUE) {

            if (CTUtils.sign(_lastVelocity) == CTUtils.sign(velocity)) {
                acceleration = Math.abs(velocity) - Math.abs(_lastVelocity);
            } else {
                acceleration = Math.abs(velocity - _lastVelocity);
            }
        } else {
            acceleration = velocity;
        }
        acceleration = Math.abs(Math.max(Math.min(acceleration, 2d), -2d));

        _LateralVelocityLast10.add(Math.abs(lateralVelocity));
        if (_LateralVelocityLast10.size() > 10)
            _LateralVelocityLast10.remove(0);

        double LatVelLast10 = 0;
        for (int k = 0; k < _LateralVelocityLast10.size(); k++) {
            LatVelLast10 += _LateralVelocityLast10.get(k);
        }

        if (direction != _lastDirection)
        {
            _lastDirectionTimeChange = (int)_robot.getTime();
        }

        double advancingVelocity = -_robot.getVelocity() * Math.cos(_radarScanner.nme.bearingRadians);
        double advVel = (double)(new Integer((int) Math.round(advancingVelocity)));
        advVel = CTUtils.clamp(0, advVel / 8, 1);

        surfData sd = new surfData();
        sd.enemyLocation = (Point2D.Double)_enemyLocation.clone();
        sd.playerLocation = new Point2D.Double(_robot.getX(), _robot.getY());
        sd.lateralVelocity = lateralVelocity;
        sd.advancingVelocity = advVel;
        sd.direction =  direction;
        sd.absBearing = new Double(absBearing + Math.PI);
        sd.acceleration = acceleration;
        sd.lateralDistanceLast10 = LatVelLast10;
        sd.timeSinceDirectionChange = (int)_robot.getTime() - _lastDirectionTimeChange;
        sd.forwardWallDistance = CTUtils.wallDistance(sd.enemyLocation.x, sd.enemyLocation.y, e.getDistance(),  absBearing + Math.PI, direction );
        sd.reverseWallDistance = CTUtils.wallDistance(sd.enemyLocation.x, sd.enemyLocation.y, e.getDistance(), absBearing + Math.PI, -direction );
        _surfData.add(0, sd);

        //System.out.println("fd: " + sd.forwardWallDistance + ", rd: " + sd.reverseWallDistance);

// Need location, lateral velocity, heading, lateral direction, advancing velocity and angle from enemy

        double bulletPower = _radarScanner._oppEnergy - e.getEnergy();
        if (bulletPower < 3.01 && bulletPower > 0.09
                && _surfData.size() > 2) {

            surfData _surf = ((surfData)_surfData.get(2));

            EnemyWave ew = new EnemyWave();
            ew.fireTime = _robot.getTime() - 1;
            ew.bulletVelocity = CTUtils.bulletVelocity(bulletPower);
            ew.distanceTraveled = CTUtils.bulletVelocity(bulletPower);
            ew.direction = _surf.direction;
            ew.directAngle = _surf.absBearing;
            ew.maxEscapeAngle = CTUtils.maxEscapeAngle(ew.bulletVelocity);
            ew.fireLocation = (Point2D.Double)_enemyLocation.clone(); // last tick

            ew.acceleration = _surf.acceleration;
            ew.lateralDistanceLast10 = _surf.lateralDistanceLast10;
            ew.timeSinceDirectionChange = _surf.timeSinceDirectionChange;
            ew.forwardWallDistance = _surf.forwardWallDistance;
            ew.reverseWallDistance = _surf.reverseWallDistance;
            ew.lateralVelocity = _surf.lateralVelocity;
            ew.advancingVelocity = _surf.advancingVelocity;

            _enemyWaves.add(ew);

            calculateShadowsForWave(ew);
        }

        _lastVelocity = velocity;
        _lastDirection = direction;

        _radarScanner._oppEnergy = e.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _enemyLocation = CTUtils.project(_myLocation, absBearing, e.getDistance());

    }

    public void update(ScannedRobotEvent e) {

        updateWaves();
        doSurfing();

        //System.out.println("Gunheat: " + _lastGunHeat);
        if (_lastGunHeat < _robot.getGunHeat())
        {
            //System.out.println("Calculating shadows...");
            bullets.add(Targeting.lastBullet);
            // calculate where it will be on all future waves
            calculateShadowsForBullet(Targeting.lastBullet);
        }
        _lastGunHeat = _robot.getGunHeat();

        int topx = 800-155, topy = 5;



        drawFactor(_surfStats, 0, _surfStats.length, "Firing Output", topx, topy, 0);

        drawFactorIndex("BEST", debugFactorIndex, topx, topy, 8);
        drawFactorIndex("CUR", debugCurrentIndex, topx, topy, 0);
    }

    public void drawFactorIndex(String name, double factorIndex, int topx, int topy, int offset)
    {
        double graphWidth = 150;
        int height = 45;
        int graphx = topx, graphy = topy + height - 10;
        //double factorIndex = getFactorIndex(ew)

        Point2D.Double nextpoint = new Point2D.Double(graphx + (int) ((double)factorIndex * (graphWidth / _surfStats.length)) - 1, graphy);

        Graphics2D g = _robot.getGraphics();
        g.setColor(new Color(1, 255, 0));
        g.drawLine((int) nextpoint.x, (int) nextpoint.y, (int) nextpoint.x, (int) nextpoint.y+height);

        g.setColor(Color.GREEN);
        g.setFont(new Font("Verdana", Font.PLAIN, 8));
        g.drawString(name, (int)nextpoint.x-8, graphy + height + 2 + offset);
    }

    public void drawFactor(double[] data, int factorStart, int featureCount, String featureName, int topx, int topy, int position) {
        Graphics2D g = _robot.getGraphics();

        double graphWidth = 150;
        int height = 45;

        topy = (height + 5) * position;
        int graphx = topx, graphy = topy + height - 10, cnt = 0;
        int rightBorder = graphx + ((int) graphWidth - (int) (graphWidth / featureCount));
        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy, rightBorder, graphy);
        g.drawLine(graphx, graphy, graphx, graphy + (height - 10));
        g.drawLine(rightBorder, graphy, rightBorder, graphy + (height - 10));

        g.setColor(Color.white);
        Point2D.Double lastpoint = new Point2D.Double(graphx, graphy);

        g.setFont(new Font("Verdana", Font.PLAIN, 10));
        g.drawString(featureName, graphx, graphy - 10);

        for (int i = factorStart; i < factorStart + featureCount && i < data.length; i++) {
            g.setColor(Color.MAGENTA);
            //g.drawOval(graphx + (int) (cnt * (graphWidth / featureCount)) - 1, graphy + (int) (data[i] * 20), 2, 2);
            Point2D.Double nextpoint = new Point2D.Double(graphx + (int) (cnt * (graphWidth / featureCount)) - 1, graphy + (int) (data[i] * (height - 10)));
            g.drawLine((int) lastpoint.x, (int) lastpoint.y, (int) nextpoint.x, (int) nextpoint.y);

            lastpoint = nextpoint;

            cnt++;
        }

        //Point2D.Double nextpoint = new Point2D.Double(graphx + (int) graphWidth, graphy);
        //g.drawLine((int)lastpoint.x, (int)lastpoint.y, (int)nextpoint.x, (int)nextpoint.y);

    }

    public double[] getInputForWave(EnemyWave w) {

        // Distance - Range 0 - 800, split into 11 features
        double bft = w.playerDistance / _radarScanner.FIRE_SPEED;
        double[] fdistance = RBFUtils.processDataIntoFeatures(Math.min(bft, 105), 105, RBFUtils.getCenters(0, 105, 11));

        // Acceleration - Range 0 - 1.0, split into 7 features
        double[] faccel = RBFUtils.processDataIntoFeatures(w.acceleration, 0.1, RBFUtils.getCenters(-2, 2, 7));

        // Lateral Velocity - Range -8 - 8, split into 8 features
        double[] flatvel = RBFUtils.processDataIntoFeatures(w.lateralVelocity, 2.0, RBFUtils.getCenters(-8, 8, 8));

        // SinceDirectionChange - Range 0 - bft, split into 7 features
        double[] fsincedirch = RBFUtils.processDataIntoFeatures(Math.min(w.timeSinceDirectionChange, bft), 1, RBFUtils.getCenters(0, bft, 7));

        double wrdf = Math.min(1.5, w.forwardWallDistance);
        double wrdb = Math.min(1.0, w.reverseWallDistance);

        // Forward radians to wall - Range 0.0 - 1.5, split into 7 features
        double[] ffwrdf = RBFUtils.processDataIntoFeatures(wrdf, 0.05, RBFUtils.getCenters(0, 1.5, 7));

        // Back radians to wall - Range 0.0 - 1.0, split into 4 features
        double[] ffwrdb = RBFUtils.processDataIntoFeatures(wrdb, 0.05, RBFUtils.getCenters(0, 1.0, 4));

        // Advancing velocity
        double[] fadvvel = RBFUtils.processDataIntoFeatures(w.advancingVelocity, 0.02, RBFUtils.getCenters(0, 1.0, 5));


        return RBFUtils.mergeFeatures(fdistance, faccel, flatvel, fsincedirch, ffwrdf, ffwrdb, fadvvel);

    }

    public Color getShade (double i)
    {
        i = 1.0 - CTUtils.clamp(i, 0.0, 1.0);

        // Heat map is red, orange, yellow, green, cyan, blue
        Color colors[] = {new Color(255, 20, 0),
                new Color(255, 112, 8),
                new Color(255, 245, 0),
                new Color(45, 255, 0),
                new Color(0, 228, 255),
                new Color(0, 74, 128)};

        return colors[(int)CTUtils.clamp(Math.round(i * colors.length), 0, colors.length - 1)];
    }

    public void drawWaves() {

        Graphics g = _robot.getGraphics();

        BestWaves best = getClosestSurfableWave();
        double[] shadows = new double[OUTPUT_LENGTH];

        Arrays.fill(shadows, 1.0);

        if (best != null && best.firstWave != null && best.firstWave.safePoints != null)
        {
            EnemyWave surfWave = best.firstWave;
            Point2D.Double pt = (Point2D.Double)surfWave.safePoints.get(surfWave.safePoints.size()-1);
            g.setColor(new Color(1, 255, 0));
            g.drawRect((int)pt.x-18, (int)pt.y-18, 36, 36);

            for (int i = 0; i < surfWave.bulletShadows.size(); i++)
            {
                double[] bs = surfWave.bulletShadows.get(i);

                int shadowStart = getFactorIndex(bs[0]);
                int shadowEnd = getFactorIndex(bs[1]);

                for (int k = shadowStart; k <= shadowEnd; k++)
                {
                    shadows[k] = 0.0;
                }
            }

        }




        for (int i = 0; i < _enemyWaves.size(); i++) {

            EnemyWave ew = (EnemyWave)_enemyWaves.get(i);


            Point2D.Double center = new Point2D.Double(ew.fireLocation.x, ew.fireLocation.y);

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);

            double angleDivision = (ew.maxEscapeAngle * 2.0 / (double)OUTPUT_LENGTH);
            int radius = (int)ew.getRadius(_robot.getTime());

            radius += ew.bulletVelocity;
            g.setColor(new Color(255, 188, 0));
            if (radius - 40 < center.distance(_myLocation))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            g.setColor(java.awt.Color.green);
            int cTime = (int) _robot.getTime();

            if (ew.waveGuessFactors != null && i == 0) {
                for (int p = 0; p < ew.waveGuessFactors.length; p++) {
                    double angleOffset = angleDivision * (double) p * 0;

                    //float shade = (float) ew.waveGuessFactors[p];
                    //shade = (float) CTUtils.clamp(shade * 10, 0.2, 1.0);
                    //g.setColor(new Color(0, shade, 1, 1.0f));
                    g.setColor(getShade(ew.waveGuessFactors[p]));

                    if (shadows[p] < 1.0)
                        g.setColor(new Color(141, 64, 169));

                    //System.out.print(shade + " ");
                    //System.out.println("DA: " + ew.directAngle + ", AD: " + angleDivision + ", MEA: " + ew.maxEscapeAngle);
                    Point2D.Double p2 = CTUtils.project(center, (ew.directAngle ) + (angleDivision * p) - (angleDivision * (OUTPUT_LENGTH / 2)), radius-ew.bulletVelocity);
                    Point2D.Double p3 = CTUtils.project(p2, (ew.directAngle) + (angleDivision * p) - (angleDivision * (OUTPUT_LENGTH / 2)), (int) (ew.bulletVelocity));

//                g.drawOval((int) (p2.x - 1), (int) (p2.y - 1), 2, 2);
                    g.drawLine((int) p2.getX(), (int) p2.getY(), (int) p3.getX(), (int) p3.getY());
                }
            }
        }
    }

    public void updateWaves() {
        Graphics g = _robot.getGraphics();


        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave)_enemyWaves.get(x);

            ew.distanceTraveled = (_robot.getTime() - ew.fireTime) * ew.bulletVelocity;
            ew.currentDistanceToPlayer = _myLocation.distance(ew.fireLocation) - ew.distanceTraveled;

            g.setColor(new Color(128, 1, 0));
            g.drawOval((int)(ew.fireLocation.x-(ew.distanceTraveled)), (int)(ew.fireLocation.y-(ew.distanceTraveled)), (int)ew.distanceTraveled*2, (int)ew.distanceTraveled*2);

            if (ew.didIntersect(_myLocation, _robot.getTime()))
            {
                logHit(ew, _myLocation, false);
                _enemyWaves.remove(x);
                x--;
            }
            /*
            if (ew.distanceTraveled >
                    _myLocation.distance(ew.fireLocation) + 50) {
                logHit(ew, _myLocation, false);
                _enemyWaves.remove(x);
                x--;
            }*/
        }

        drawWaves();

    }

    public BestWaves getClosestSurfableWave() {
        double closestDistance = 50000; // I juse use some very big number here
        EnemyWave surfWave = null;
        EnemyWave surfWave2 = null;

        Collections.sort(_enemyWaves);

        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave)_enemyWaves.get(x);
            double distance = ew.currentDistanceToPlayer;

            if (distance > ew.bulletVelocity) {
                surfWave = ew;

                if (x+1 < _enemyWaves.size())
                    surfWave2 = _enemyWaves.get(x+1);

                break;
            }
        }
        /*
        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave)_enemyWaves.get(x);
            double distance = _myLocation.distance(ew.fireLocation)
                    - ew.distanceTraveled;

            if (distance > ew.bulletVelocity && distance < closestDistance) {
                surfWave2 = surfWave;
                surfWave = ew;
                closestDistance = distance;
            }
        }
        */

        if (surfWave != null) {
            BasicMLData inp = new BasicMLData(getInputForWave(surfWave));
            surfWave.waveGuessFactors = basicNetwork.compute(inp).getData();
            _surfStats = surfWave.waveGuessFactors;
        }


        if (surfWave2 != null) {
            BasicMLData inp = new BasicMLData(getInputForWave(surfWave2));
            surfWave2.waveGuessFactors = basicNetwork.compute(inp).getData();
        }
        //System.out.println("Guessfactor Output: " + Arrays.toString(_surfStats));
        int wc = ((surfWave == null ? 0 : 1) + (surfWave2 == null ? 0 : 1));

        return new BestWaves(surfWave, surfWave2);
    }

    public static double getGuessFactor(EnemyWave ew, Point2D.Double targetLocation) {
        double offsetAngle = (CTUtils.absoluteBearing(ew.fireLocation, targetLocation)
                - ew.directAngle);
        double factor = Utils.normalRelativeAngle(offsetAngle)
                / CTUtils.maxEscapeAngle(ew.bulletVelocity) * ew.direction;

        return factor;
    }

    // Given the EnemyWave that the bullet was on, and the point where we
// were hit, calculate the index into our stat array for that factor.
    public static int getFactorIndex(EnemyWave ew, Point2D.Double targetLocation) {

        double factor = getGuessFactor(ew, targetLocation);

        return getFactorIndex(factor);
    }

    public static int getFactorIndex(double factor)
    {
        return (int)CTUtils.limit(0,
                (factor * ((BINS - 1) / 2)) + ((BINS - 1) / 2),
                BINS - 1);
    }

    // Given the EnemyWave that the bullet was on, and the point where we
// were hit, update our stat array to reflect the danger in that area.
    public void logHit(EnemyWave ew, Point2D.Double targetLocation, boolean isHit) {

        if (!isHit)
            return;  // Bail.. don't log hits that aren't real

        double enemyX = ew.fireLocation.getX(), enemyY = ew.fireLocation.getY();
        double startX = targetLocation.getX(), startY = targetLocation.getY();

        double gf = getGuessFactor(ew, targetLocation);
        double gfwidth = CTUtils.botWidthAimAngle(Math.sqrt((enemyX-startX)*(enemyX-startX) + (enemyY-startY)*(enemyY-startY) ));

        double[] centers = RBFUtils.getCenters(-1.0, 1.0, OUTPUT_LENGTH);
        double[] ideal = RBFUtils.processDataIntoFeatures(gf, gfwidth*5, centers);

        _theData.clear();
        _theData.add(new BasicMLDataPair(new BasicMLData(getInputForWave(ew)), new BasicMLData(ideal)));

        for (int i = 0; i < _lastHitsData.size(); i++)
        {
            _theData.add(_lastHitsData.get(i));
        }

        basicTrain.iteration(2);

        if (isHit && 2==3)
        {
            _lastHitsData.add(_theData.get(_theData.size()-1));

            if (_lastHitsData.size() > 5)
                _lastHitsData.remove(0);
        }

    }

    public void onBulletHitBullet(BulletHitBulletEvent e)
    {

        if (!_enemyWaves.isEmpty()) {
            Point2D.Double hitBulletLocation = new Point2D.Double(
                    e.getBullet().getX(), e.getBullet().getY());
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++) {
                EnemyWave ew = (EnemyWave)_enemyWaves.get(x);

                if (Math.abs(ew.distanceToPoint(hitBulletLocation) - ew.distanceTraveled) < 50) {
                    hitWave = ew;
                    break;
                }
            }

            if (hitWave != null) {
                logHit(hitWave, hitBulletLocation, true);
                //hitWave.shadowBullet();
                hitWave.collidedWithBullet = true;
                // We can remove this wave now, of course.
                // _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
        }

    }

    public void onHitByBullet(HitByBulletEvent e) {

        // If the _enemyWaves collection is empty, we must have missed the
        // detection of this wave somehow.
        if (!_enemyWaves.isEmpty()) {

            removeShadow(e.getBullet());

            Point2D.Double hitBulletLocation = new Point2D.Double(
                    e.getBullet().getX(), e.getBullet().getY());
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++) {
                EnemyWave ew = (EnemyWave)_enemyWaves.get(x);

                if (Math.abs(ew.distanceTraveled -
                        _myLocation.distance(ew.fireLocation)) < 50
                        && Math.abs(CTUtils.bulletVelocity(e.getBullet().getPower())
                        - ew.bulletVelocity) < 0.001) {
                    hitWave = ew;
                    break;
                }
            }

            if (hitWave != null) {
                logHit(hitWave, hitBulletLocation, true);

                // We can remove this wave now, of course.
                _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
        }

    }

    private void removeShadow(final Bullet b) {
        for(final EnemyWave wave : _enemyWaves) {
            // check if bullet has yet to pass through wave
            final double r = wave.getRadius(_robot.getTime());
            final double d = wave.fireLocation.distanceSq(_myLocation) - r * r;
            if(d > _myLocation.distanceSq(b.getX(), b.getY())) // if it hasn't,
                // remove it
                // from the wave
                wave.removeShadow(b);
        }
    }

    // Calculate the path a robot can take with smoothing..
    public ArrayList<RobotState> calculateFutureMoves (ArrayList<EnemyWave> waves, double x, double y, double absBearingRadians, double velocity, double maxVelocity, double heading, double attackAngle, boolean clockwise, long currentTime, Rectangle2D.Double battleField,	double bfWidth, double bfHeight,
                                               double wallStick, boolean ignoreWallHits)
    {
        int index = 0;
        EnemyWave surfWave = waves.get(0);

        ArrayList<RobotState> moves = new ArrayList<RobotState>();

        MovSim mv = CTUtils.getMovSim();



        double wallDanger = 0;
        Rectangle2D.Double field = new Rectangle2D.Double(_radarScanner._fieldRect.x+20,_radarScanner._fieldRect.y+20,
                _radarScanner._fieldRect.width-40,_radarScanner._fieldRect.height-40);

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do {

            RobotState next = CTUtils.nextPerpendicularWallSmoothedLocation(
                    new Point2D.Double(x,y), absBearingRadians,velocity, maxVelocity, heading,
                    attackAngle, clockwise, currentTime,
                    battleField, bfWidth, bfHeight,
                    wallStick, ignoreWallHits);


            Point2D.Double predictedPosition = next.location;
            double predictedDistance = surfWave.distanceToPoint(predictedPosition);
            int ticks = CTUtils.bulletTicks(predictedDistance - surfWave.distanceTraveled ,surfWave.bulletPower) - 2;

            //double dgs = calculateScaledDanger (surfWave, predictedPosition);


            //if (!field.contains(next.location))
            //    wallDanger = 0.1; // BEST IS = 0.1

            //for (int z = 1; z < waves.size(); z++)
            //    dgs += calculateScaledDanger (waves.get(z), predictedPosition); // * (1.0/(z));


            /*
            dgs += 5*(1/_radarScanner.nme.distance);
            dgs += 5*(1- 1/Point2D.distance(predictedPosition.getX(),predictedPosition.getY(),_radarScanner._fieldRect.getCenterX(), _radarScanner._fieldRect.getCenterY()));
            */
            //dgs *= (1+surfWave.bulletPower);
            //dgs += wallDanger;   // Need to somehow manage positioning better.. walls are death

            next.danger = 0; //dgs;
            next.tickDistance = ticks;
            //next.direction = direction;

            x = next.location.getX();
            y = next.location.getY();
            heading = next.heading;
            velocity = next.velocity;
            absBearingRadians = CTUtils.absoluteBearing(surfWave.fireLocation, next.location);

            moves.add(next);

            counter++;

            if (next.location.distance(surfWave.fireLocation) - 20 <
                    surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                //   + surfWave.bulletVelocity
                    ) {
                intercepted = true;
            }
        } while(!intercepted && counter < 500);

        //we can't get the the last point, because we need to slow down
        if(moves.size() > 1)
            moves.remove(moves.size() - 1);

        return moves;
    }

    // CREDIT: mini sized predictor from Apollon, by rozu
// http://robowiki.net?Apollon
    public ArrayList predictPositions(EnemyWave surfWave, int direction) {
        /*
        Point2D.Double predictedPosition = (Point2D.Double)_myLocation.clone();
        double predictedVelocity = _robot.getVelocity();
        double predictedHeading = _robot.getHeadingRadians();
        */
        double maxTurning, moveAngle, moveDir;
        ArrayList traveledPoints = new ArrayList();

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do {
            double distance = surfWave.predictedPosition.distance(surfWave.fireLocation);
            double offset = Math.PI/2 - 1 + distance/400;

            moveAngle =
                    CTUtils.wallSmoothing(_fieldRect, _robot.getBattleFieldWidth(), _robot.getBattleFieldHeight(),
                                        surfWave.predictedPosition,  CTUtils.absoluteBearing(surfWave.fireLocation,
                                    surfWave.predictedPosition) + (direction * (offset)), direction, WALL_STICK)
                                    - surfWave.predictedHeading;
            /*
                    GTSurferMove.wallSmoothing(predictedPosition, CTUtils.absoluteBearing(surfWave.fireLocation,
                            predictedPosition) + (direction * (offset)), direction)
                            - predictedHeading;*/
            moveDir = 1;

            if(Math.cos(moveAngle) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            moveAngle = Utils.normalRelativeAngle(moveAngle);

            // maxTurning is built in like this, you can't turn more then this in one tick
            maxTurning = Math.PI/720d*(40d - 3d*Math.abs(surfWave.predictedVelocity));
            surfWave.predictedHeading = Utils.normalRelativeAngle(surfWave.predictedHeading
                    + CTUtils.limit(-maxTurning, moveAngle, maxTurning));

            // this one is nice ;). if predictedVelocity and moveDir have
            // different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
            surfWave.predictedVelocity += (surfWave.predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
            surfWave.predictedVelocity = CTUtils.limit(-8, surfWave.predictedVelocity, 8);

            // calculate the new predicted position
            surfWave.predictedPosition = CTUtils.project(surfWave.predictedPosition, surfWave.predictedHeading, surfWave.predictedVelocity);

            //add this point the our prediction
            traveledPoints.add(surfWave.predictedPosition);

            counter++;

            if (surfWave.predictedPosition.distance(surfWave.fireLocation) - 20 <
                    surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                //   + surfWave.bulletVelocity
                    ) {
                intercepted = true;
            }
        } while(!intercepted && counter < 500);

        //we can't get the the last point, because we need to slow down
        if(traveledPoints.size() > 1)
            traveledPoints.remove(traveledPoints.size() - 1);

        return traveledPoints;
    }

    public void analyzePath(BestWaves best)
    {
        Point2D.Double p1 = getBestPoint(best.firstWave, (Point2D.Double)_myLocation.clone(), _robot.getVelocity(), _robot.getHeadingRadians());

        Graphics g = _robot.getGraphics();
        g.setColor(Color.RED);
        g.drawOval((int)p1.x, (int)p1.y, 4, 4);

        Point2D.Double pdest = (Point2D.Double)best.firstWave.safePoints.get(best.firstWave.safePoints.size()-1);
        double distRemain = pdest.distance(_myLocation);
        int bulletTicks = CTUtils.bulletTicks(best.firstWave.currentDistanceToPlayer, best.firstWave.bulletPower);
        int tripTicks = best.firstWave.safePoints.size();

        System.out.println("Distance to pdest: " + distRemain + ", bullet Ticks: " + CTUtils.bulletTicks(best.firstWave.currentDistanceToPlayer, best.firstWave.bulletPower) + ", safepoint cnt: " + best.firstWave.safePoints.size());

        Point2D.Double endp1 = (Point2D.Double)best.firstWave.safePoints.get(best.firstWave.safePoints.size()-1);

        g.setColor(new Color(150, 0, 255));
        g.drawOval((int)endp1.x, (int)endp1.y, 2, 2);

        if (tripTicks < bulletTicks && bulletTicks < 100)
        {
            // We have extra ticks, what can we do with them?
            int extraTicks = bulletTicks - tripTicks;

            if (best.secondWave != null) {

                Point2D.Double temp = (Point2D.Double) _myLocation.clone();
                _myLocation = endp1;
                Point2D.Double p2 = getBestPoint(best.secondWave, (Point2D.Double) best.firstWave.predictedPosition, best.firstWave.predictedVelocity, best.firstWave.predictedHeading);
                _myLocation = temp;
            }
        }
    }

    public double checkDanger(EnemyWave surfWave, Point2D.Double position) {
        int index = getFactorIndex(surfWave, position);
        double distance = position.distance(surfWave.fireLocation);
        return surfWave.waveGuessFactors[index]/distance;
    }

    // Calculates the average danger over a particular span of guess factors
    public double checkDangerSpan(EnemyWave surfWave, Point2D.Double position, int totalSpan)
    {
        double[] shadows = new double[OUTPUT_LENGTH];
        double gf = getGuessFactor(surfWave, position);
        int index = getFactorIndex(gf);
        int halfSpan = (int)Math.max(1.0, Math.ceil(totalSpan/2.0));
        int startIndex = (int)CTUtils.clamp(index-halfSpan, 0, index);
        int endIndex = (int)CTUtils.clamp(index+halfSpan, index, OUTPUT_LENGTH-1);

        boolean hasShadows = false;

        Arrays.fill(shadows, 1.0);

        for (int i = 0; i < surfWave.bulletShadows.size(); i++)
        {
            double[] bs = surfWave.bulletShadows.get(i);

            int shadowStart = getFactorIndex(bs[0]);
            int shadowEnd = getFactorIndex(bs[1]);

            //System.out.println("shadowStart: " + shadowStart + ", end: " + shadowEnd);
            //System.out.println("Gf: " + gf + ", shadow.min: " + bs[0] + ", shadow.max: " + bs[1]);

            for (int k = shadowStart; k <= shadowEnd; k++)
            {
                //hasShadows = true;
                shadows[k] = 0.0;
                surfWave.waveGuessFactors[k] = 0;
            }
        }

        //if (hasShadows)
        //    System.out.println(Arrays.toString(shadows));

        double danger = 0.0;
        for (int i = startIndex; i <= endIndex; i++)
        {
            danger += surfWave.waveGuessFactors[i] * shadows[i];

        }
        danger /= totalSpan;


        //danger *= position.distance(surfWave.fireLocation) - surfWave.distanceTraveled;

        return danger;
    }

    public Point2D.Double getBestPoint(EnemyWave surfWave, Point2D.Double startPosition, double startVelocity, double startHeading){

        surfWave.predictedPosition = startPosition;
        surfWave.predictedVelocity = startVelocity;
        surfWave.predictedHeading = startHeading;

        double distance = surfWave.distanceTraveled;

        // Calculate how much of the wave we need to be concerned with
        double botWidthAimAngle = CTUtils.botWidthAimAngle(distance);

        double gfSpan = 2.0 / (double)OUTPUT_LENGTH; // -1.0 to 1.0 is a 2.0 span over OUTPUT_LENGTH factors
        int totalSpan = (int)Math.max(Math.ceil(botWidthAimAngle / gfSpan), 1.0);
        //System.out.println("Span: " + totalSpan + " gfSpan: " + gfSpan + ", botWidthAimAngle: " + botWidthAimAngle);
        Graphics g = _robot.getGraphics();
        if(surfWave.safePoints == null){

            ArrayList forwardPoints = predictPositions(surfWave, 1);

            // Store copies of all the predicted final spots before we overwrite them inside the wave
            double pheading = surfWave.predictedHeading;
            double pvelocity = surfWave.predictedVelocity;
            Point2D.Double pposition = surfWave.predictedPosition;

            // Now reset all the original starting spots
            surfWave.predictedHeading = startHeading;
            surfWave.predictedPosition = startPosition;
            surfWave.predictedVelocity = startVelocity;

            ArrayList reversePoints = predictPositions(surfWave, -1);

            int FminDangerIndex = 0;
            int RminDangerIndex = 0;
            double FminDanger = Double.POSITIVE_INFINITY;
            double RminDanger = Double.POSITIVE_INFINITY;
            for(int i = 0, k = forwardPoints.size(); i < k; i++){

                Point2D.Double goToPoint = (Point2D.Double) (forwardPoints.get(i));

                g.setColor(new Color(253, 242, 77));
                g.drawOval((int)goToPoint.x,(int)goToPoint.y, 4, 4);

                double thisDanger = checkDangerSpan(surfWave, (Point2D.Double) (forwardPoints.get(i)), totalSpan);
                //double thisDanger = checkDanger(surfWave, (Point2D.Double) (forwardPoints.get(i)));

                if(thisDanger <= FminDanger){
                    FminDangerIndex = i;
                    FminDanger = thisDanger;
                }
            }
            for(int i = 0, k = reversePoints.size(); i < k; i++){

                Point2D.Double goToPoint = (Point2D.Double) (reversePoints.get(i));

                g.setColor(new Color(253, 242, 77));
                g.drawOval((int)goToPoint.x,(int)goToPoint.y, 4, 4);

                double thisDanger = checkDangerSpan(surfWave, (Point2D.Double) (reversePoints.get(i)), totalSpan);
                //double thisDanger = checkDanger(surfWave, (Point2D.Double) (reversePoints.get(i)));

                if(thisDanger <= RminDanger){
                    RminDangerIndex = i;
                    RminDanger = thisDanger;
                }
            }
            ArrayList bestPoints;
            int minDangerIndex;

            if(FminDanger < RminDanger ){
                surfWave.predictedPosition = pposition;
                surfWave.predictedHeading = pheading;
                surfWave.predictedVelocity = pvelocity;

                bestPoints = forwardPoints;
                minDangerIndex = FminDangerIndex;
            }
            else {
                bestPoints = reversePoints;
                minDangerIndex = RminDangerIndex;
            }

            Point2D.Double bestPoint = (Point2D.Double)bestPoints.get(minDangerIndex);

            while(bestPoints.indexOf(bestPoint) != -1)
                bestPoints.remove(bestPoints.size() - 1);
            bestPoints.add(bestPoint);

            surfWave.safePoints = bestPoints;

            //debugging - so that we should always be on top of the last point
            //bestPoints.add(0,new Point2D.Double(_myLocation.x, _myLocation.y));
            bestPoints.add(0, startPosition.clone());
        }
        else
        if(surfWave.safePoints.size() > 1)
            surfWave.safePoints.remove(0);


        if(surfWave.safePoints.size() >= 1){
            for(int i = 0,k=surfWave.safePoints.size(); i < k; i++){
                Point2D.Double goToPoint = (Point2D.Double)surfWave.safePoints.get(i);

                g.setColor(new Color(253, 242, 77));
                g.drawOval((int)goToPoint.x,(int)goToPoint.y, 4, 4);

                if(goToPoint.distanceSq(_myLocation) > 20*20*1.1) {
                    //System.out.println("goToPoint.dist=" + goToPoint.distance(_myLocation) + ", gtp size: " + surfWave.safePoints.size());
                    //if it's not 20 units away we won't reach max velocity
                    return goToPoint;
                }
            }
            //if we don't find a point 20 units away, return the end point
            return (Point2D.Double)surfWave.safePoints.get(surfWave.safePoints.size() - 1);


        }

        return null;
    }

    public void doSurfing() {
        int buffer = 36*2;
        Rectangle2D.Double _safeZone
                = new java.awt.geom.Rectangle2D.Double(buffer, buffer, 800-(buffer*2), 600-(buffer*2));

        BestWaves best = getClosestSurfableWave();
        EnemyWave surfWave = best.firstWave;
        double distance = _enemyLocation.distance(_myLocation);
        if (surfWave == null || distance < 50) {
            //do 'away' movement  best distance of 400 - modified from RaikoNano
            double absBearing = CTUtils.absoluteBearing(_myLocation, _enemyLocation);
            double headingRadians = _robot.getHeadingRadians();
            double stick = 160;//Math.min(160,distance);
            double  v2, offset = Math.PI/2 + 1 - distance/400;

            while(!_fieldRect.
                    contains(CTUtils.project(_myLocation, v2 = absBearing + direction * (offset -= 0.02), stick)

                            // 	getX() + stick * Math.sin(v2 = absBearing + direction * (offset -= .02)), getY() + stick * Math.cos(v2)
                    ));


            if( offset < Math.PI/3 )
                direction = -direction;
            _robot.setAhead(50 * Math.cos(v2 - headingRadians));
            _robot.setTurnRightRadians(Math.tan(v2 - headingRadians));

        }
        else {
            /*
                        predictedPosition = (Point2D.Double)_myLocation.clone();
            predictedVelocity = _robot.getVelocity();
            predictedHeading = _robot.getHeadingRadians();
             */
            Point2D.Double p1 = getBestPoint(best.firstWave, (Point2D.Double)_myLocation.clone(), _robot.getVelocity(), _robot.getHeadingRadians());


            Graphics g = _robot.getGraphics();
            g.setColor(Color.RED);
            g.drawOval((int)p1.x, (int)p1.y, 4, 4);

            Point2D.Double pdest = (Point2D.Double) best.firstWave.safePoints.get(best.firstWave.safePoints.size()-1);
            double distRemain = pdest.distance(_myLocation);

            double[][] corners = best.firstWave.getCorners(new Rectangle2D.Double(pdest.x-18, pdest.y-18, 36, 36));
            double wd1 = best.firstWave.fireLocation.distance(new Point2D.Double(corners[0][0], corners[0][1])) - best.firstWave.distanceTraveled;
            double wd2 = best.firstWave.fireLocation.distance(new Point2D.Double(corners[1][0], corners[1][1])) - best.firstWave.distanceTraveled;
            double wd3 = best.firstWave.fireLocation.distance(new Point2D.Double(corners[2][0], corners[2][1])) - best.firstWave.distanceTraveled;
            double wd4 = best.firstWave.fireLocation.distance(new Point2D.Double(corners[3][0], corners[3][1])) - best.firstWave.distanceTraveled;

//            int bulletTicks = CTUtils.bulletTicks(best.firstWave.currentDistanceToPlayer, best.firstWave.bulletPower);
            int bulletTicks = CTUtils.bulletTicks(Math.min(Math.min(wd1,wd2), Math.min(wd3,wd4)), best.firstWave.bulletPower);
            int tripTicks = best.firstWave.safePoints.size();

            System.out.println("Original bullet ticks: " + CTUtils.bulletTicks(best.firstWave.currentDistanceToPlayer, best.firstWave.bulletPower) +
                               ", New: " + bulletTicks);

            //System.out.println("Distance to pdest: " + distRemain + ", bullet Ticks: " + CTUtils.bulletTicks(best.firstWave.currentDistanceToPlayer, best.firstWave.bulletPower) + ", safepoint cnt: " + best.firstWave.safePoints.size());
            //System.out.println("Vel: " + best.firstWave.predictedVelocity + ", size: " + best.firstWave.safePoints.size());

            Point2D.Double endp1 = (Point2D.Double)best.firstWave.safePoints.get(best.firstWave.safePoints.size()-1);
            Point2D.Double endp2 = CTUtils.project(best.firstWave.fireLocation, best.firstWave.absoluteBearing(endp1), endp1.distance(best.firstWave.fireLocation) + 25);

            g.setColor(new Color(150, 0, 255));
            g.drawOval((int)endp1.x, (int)endp1.y, 2, 2);

            //g.setColor(new Color(255, 0, 207));
            //g.drawOval((int)endp2.x, (int)endp2.y, 2, 2);

            debugFactorIndex = getFactorIndex(best.firstWave, endp1);
            debugCurrentIndex = getFactorIndex(best.firstWave, _myLocation);


            if (tripTicks < bulletTicks && bulletTicks < 100)
            {
                // We are going to have some time to spare!!
                double totalDist = 0;
                Point2D.Double start = _myLocation;
                for (int p = 0; p < best.firstWave.safePoints.size(); p++) {
                    Point2D.Double end = (Point2D.Double) best.firstWave.safePoints.get(p);
                    totalDist += start.distance(end);

                    start = end;
                }

                // So now we have to traverse totalDist pixels traveling up to 8 pixels a tick
                int tickDiff = bulletTicks - tripTicks;
                double extraPixels = (tickDiff) * 8.0; // This isn't right because of accel/decel
                double desiredDistance = totalDist + extraPixels;

                ArrayList<Point2D.Double> safePoints = new ArrayList<Point2D.Double>();

                if (best.firstWave.redirected == false) {

                    int spsize = best.firstWave.safePoints.size();
                    double spdist = 0.0;
                    double offset = 1.0;
                    int iterations = 0;

                    do {
                        spdist = 0.0;
                        offset += 0.1;

                        //if (offset > extraPixels)
                        //    break;
                        if (iterations++ > 200)
                            break;

                        Point2D.Double lastp = _myLocation;
                        safePoints.clear();

                        for (int p = 0; p < spsize; p++) {
                            Point2D.Double sp = (Point2D.Double) best.firstWave.safePoints.get(p);
                            Point2D.Double sp2 = CTUtils.project(best.firstWave.fireLocation, best.firstWave.absoluteBearing(sp), sp.distance(best.firstWave.fireLocation) + (offset * ((double) p / spsize)));

                            g.drawOval((int) sp2.x, (int) sp2.y, 2, 2);
                            spdist += sp2.distance(lastp);

                            safePoints.add(sp2);
                            lastp = sp2;
                        }
                        //safePoints.add(endp2);
                        //spdist += lastp.distance(endp2);
                        endp2 = lastp;


                    } while ((spdist + 4.0) < desiredDistance);

                    //System.out.println("Current Distance: " + totalDist + ", Offset " + offset + ", New Distance: " + spdist + ", Desired Distance: " + desiredDistance);
                }

                g.setColor(new Color(255, 0, 207));
                g.drawOval((int)endp2.x, (int)endp2.y, 2, 2);

                double d1 = endp1.distance(_radarScanner.nme.location);
                double d2 = endp2.distance(_radarScanner.nme.location);
                double cornerDist = CTUtils.cornerDistance(_myLocation, 800,600);

                if (best.firstWave.redirected == false && safePoints.size() > 0
                        && _safeZone.contains(endp2)
                        && cornerDist > 100
                        && Math.abs(d1-350) > Math.abs(d2-350)
                        )
                {
                    best.firstWave.safePoints = safePoints;
                    p1 = getBestPoint(best.firstWave, (Point2D.Double)_myLocation.clone(), _robot.getVelocity(), _robot.getHeadingRadians());

                    /*
                    bulletTicks = CTUtils.bulletTicks(best.firstWave.distanceTraveled - best.firstWave.fireLocation.distance(endp2), best.firstWave.bulletPower);
                    tripTicks = best.firstWave.safePoints.size();
                    tickDiff = bulletTicks - tripTicks;
                    extraPixels = tickDiff * 8.0;
                    */
                    best.firstWave.redirected = true;
                    System.out.println("8888888888888888888888888888888888888888888888888888888888");
                }

                    // We need to slow down..
                if (!surfWave.redirected)
                {
                    double reducePerTick = extraPixels / tripTicks;
                    _robot.setMaxVelocity(8.0 - reducePerTick);
                }


            }
            else
                _robot.setMaxVelocity(8);


            goTo(p1);
        }
    }

    private void goTo(Point2D.Double destination) {
        if(destination == null){
            if(_lastGoToPoint != null)
                destination = _lastGoToPoint;
            else
                return;
        }

        _lastGoToPoint = destination;
        Point2D.Double location = new Point2D.Double(_robot.getX(), _robot.getY());
        double distance = location.distance(destination);
        double angle = Utils.normalRelativeAngle(CTUtils.absoluteBearing(location, destination) - _robot.getHeadingRadians());
        if (Math.abs(angle) > Math.PI/2) {
            distance = -distance;
            if (angle > 0) {
                angle -= Math.PI;
            }
            else {
                angle += Math.PI;
            }
        }

        //this is hacked so that the bot doesn't turn once we get to our destination
        _robot.setTurnRightRadians(angle * Math.signum(Math.abs((int) distance)));

        _robot.setAhead(distance);
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

}
