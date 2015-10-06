package cg;


import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;

import robocode.*;
import robocode.util.Utils;

/**
 * In charge of collecting data from the radar on scanned events - RadarScanner will act as the central
 * data store for the robot
 */
public class RadarScanner {

    public final static boolean LOG_TO_FILE = true;
    public PrintStream fileWriter = null;

    private AdvancedRobot _robot;
    public ArrayList<Integer> _surfDirections;  // Public.. for now
    public ArrayList<Double> _surfAbsBearings;  // Public.. for now
    private ArrayList<Situation> _nodeQueue;
    private ArrayList<EnemySituation> _enemyNodeQueue;

    private ArrayList<BaseGun> _registeredGuns;
    //    public static Rectangle2D.Double _fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public final static int wallwidth = 18; // 18
    public static Rectangle2D.Double _fieldRect = new java.awt.geom.Rectangle2D.Double(wallwidth, wallwidth, 800 - wallwidth * 2, 600 - wallwidth * 2);


    public static double FIRE_POWER = 2;
    public static double FIRE_SPEED = 20 - FIRE_POWER * 3;

    // Let's store some data points!!
    public final static int SITUATION_DIMENSIONS = 6;

    public Enemy nme = new Enemy();

    // We must keep track of the enemy's energy level to detect EnergyDrop,
    // indicating a bullet is fired
    public static double _oppEnergy = 100.0;
    public ScannedRobotEvent _lastScan = null;

    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _lastLocation;
    public double _lastHeading = 0;
    public double _lastVelocity = 0;
    public double _lastVelocityChange = 0;
    public double _lastLatVel = 0;


    public RadarScanner() {
        _surfDirections = new ArrayList<Integer>();
        _surfAbsBearings = new ArrayList<Double>();
        _nodeQueue = new ArrayList<Situation>(100);
        _enemyNodeQueue = new ArrayList<EnemySituation>(100);
        _registeredGuns = new ArrayList<BaseGun>();

    }

    public void onBattleEnded() {
        fileWriter.close();
    }

    public void writeLogLine(Situation s) {
        // { LateralVelocity, Acceleration, NormalizedDistance, WallTriesForward, WallTriesBack, AdvancingVelocity };

        double[] point = s.getPoint();
        String output = "";

        output += s.GuessFactor + ",";
        output += s.GuessFactorChosen;

        for (int i = 0; i < point.length; i++) {
            if (output.length() != 0)
                output += ",";

            output += point[i];
        }

        fileWriter.println(output);

        //if (fileWriter.checkError())
        //    System.out.println("I could not write the count!");


    }

    public void setRobot(AdvancedRobot robot) {
        _robot = robot;

        if (fileWriter == null) {
            try {
                fileWriter = new PrintStream(new RobocodeFileOutputStream(_robot.getDataFile("count.csv")));
            } catch (IOException e) {
                System.out.println("Unable to write to variables log. " + e.getMessage());
            }
        }
    }

    public void registerGun(BaseGun gun) {
        _registeredGuns.add(gun);
    }

    public void onRoundEnded(RoundEndedEvent event) {
        nme.energy = 100;
    }

    public Situation processScanEvent(ScannedRobotEvent e, boolean turnRadar) {
        if (_lastScan == null) {
            _lastScan = e;
            return null;
        }
        _lastScan = e;

        _myLocation = new Point2D.Double(_robot.getX(), _robot.getY());

        double lateralVelocity = _robot.getVelocity() * Math.sin(e.getBearingRadians()); // Player's lateral velocity
        double enemyHeading = _robot.getHeadingRadians() + _lastScan.getBearingRadians();
        double relativeHeading = _lastScan.getHeadingRadians() - enemyHeading;
        double heading = _lastScan.getHeadingRadians();
        double x = _robot.getX();
        double y = _robot.getY();
        double distance = _lastScan.getDistance();
        long time = _robot.getTime();

        //calculate indexes for enemy
        double velocity = _lastScan.getVelocity();
        double absVelocity = Math.abs(velocity);
        double enemyLateralVelocity = velocity * Math.sin(relativeHeading);
        double advancingVelocity = -Math.cos(relativeHeading) * velocity;
        double direction = lateralVelocity < 0 ? -1 : 1;

        double energy = _robot.getEnergy();
        //double shotPower = CrushTurtle.FIRE_POWER;

        double absBearing = e.getBearingRadians() + _robot.getHeadingRadians();

        double angle = Math.toRadians((_robot.getHeading() + e.getBearing()) % 360);
        double enemyX = (_myLocation.getX() + Math.sin(angle) * e.getDistance());
        double enemyY = (_myLocation.getY() + Math.cos(angle) * e.getDistance());

        nme.enemyBox.setFrame(enemyX - 18, enemyY - 18, 36, 36);

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

        _lastVelocityChange++;
        if (Math.abs(_lastVelocity - velocity) > 0.1) {
            _lastVelocityChange = 0;
        }
        double velocityChangeValue = Math.min(_lastVelocityChange / (distance / 14.3d), 4d);

        //wall distance forward
        double wallTries = getWallTries(enemyHeading, direction, x, y, distance);
        double wallTriesBack = getWallTries(enemyHeading, -direction, x, y, distance);

        enemyLateralVelocity = Math.abs(enemyLateralVelocity);  // Two lateral velocities seem to be going on here..

        nme.lastlocation = nme.location;
        nme.location = new Point2D.Double(enemyX, enemyY);
        nme.name = e.getName();
        nme.energy = _lastScan.getEnergy();  // Don't update enemy energy with current scan
        nme.velocity = e.getVelocity();
        nme.bearing = e.getBearing();
        nme.bearingRadians = e.getBearingRadians();
        nme.heading = e.getHeading();
        nme.headingRadians = e.getHeadingRadians();
        nme.distance = e.getDistance();


        Situation scan = new Situation();
        scan.Time = time - 1;
        scan.LateralVelocity = enemyLateralVelocity / 8d;
        scan.PlayerLateralVelocity = lateralVelocity / 8d;
        scan.AdvancingVelocity = advancingVelocity / 16d;
        scan.WallTriesForward = wallTries / 20d;
        scan.WallTriesBack = wallTriesBack / 20d;
        scan.NormalizedDistance = distance / 800d;
        scan.Distance = distance;
        scan.Velocity = absVelocity / 8d;
        scan.Acceleration = acceleration / 2d;
        scan.SinceVelocityChange = velocityChangeValue / 4d;
        scan.Direction = direction;
        scan.EnemyHeading = enemyHeading;
        scan.RX = x;
        scan.RY = y;
        if (FIRE_POWER > 0) {
            scan.setBulletVelocity(FIRE_POWER);
        } else {
            scan.setBulletVelocity(1.90d);
        }


        _nodeQueue.add(scan);

        for (BaseGun gun : _registeredGuns) {
            gun.checkVirtualBullets(time, nme.enemyBox);
        }

        for (int i = _nodeQueue.size(); --i >= 0; ) {
            Situation s = _nodeQueue.get(i);
            if (s.getDistance(time) > distance - s.BulletVelocity * 0.5d) // If we passed the enemy
            {
                if (!s.setBearing(enemyX, enemyY)) {
                    // add scan to tree
                    //ssituations.addPoint(s.getPoint(), s);

                    // DO WE WANT TO PROCESS THE SITUATION IN SOME WAY BEFORE WE DISCARD IT?
                    // IF SO.. DO IT HERE


                    if (LOG_TO_FILE) {
                        writeLogLine(s);
                    } else
                        System.out.println("UNABLE TO WRITE LINE");

                    //System.out.println("Added scan point");
                    for (BaseGun gun : _registeredGuns) {
                        gun.addSituation(s);
                    }
                }

                if (s.getDistance(time) > (distance + 30d)) {
                    _nodeQueue.remove(i);

                    // remove scan from gun
                    for (BaseGun gun : _registeredGuns) {
                        gun.removePassed(s);
                    }
                }
            }
        }

		/*
        MaxHeap<Situation> similarSituations = null;
		if (situations.size() > 0)
		{
			similarSituations = situations.findNearestNeighbors(scan.getPoint(), 20, distanceFunction);
		}
		*/

        if (turnRadar) {
            double radarTurn = Utils.normalRelativeAngle(absBearing - _robot.getRadarHeadingRadians());
            double extraTurn = Math.min(Math.atan(65.0 / e.getDistance()), Rules.RADAR_TURN_RATE_RADIANS);
            radarTurn += (radarTurn < 0 ? -extraTurn : extraTurn);

            _robot.setTurnRadarRightRadians(radarTurn);
        }

        _surfDirections.add(0, new Integer((lateralVelocity >= 0) ? 1 : -1));
        _surfAbsBearings.add(0, new Double(absBearing + Math.PI));


        // RESPONSIBILITY OF RadarScanner should end here


        // gun code would go here...

        // SNIPPED!   KNN Gun code


        // postProcessScanEvent ((ScannedRobotEvent e)

        return scan;
    }

    public void postProcessScanEvent(ScannedRobotEvent e) {
        double lateralVelocity = _robot.getVelocity() * Math.sin(e.getBearingRadians()); // Player's lateral velocity

        _lastLocation = _myLocation;
        _lastVelocity = _lastScan.getVelocity();
        _lastVelocityChange = Math.min(_lastVelocityChange / (_lastScan.getDistance() / 14.3d), 4d);
        _lastLatVel = _robot.getVelocity() * Math.sin(e.getBearingRadians()); // Player's lateral velocity

        _lastScan = e;
    }


    public Situation getInterceptingSituation(double bulletX, double bulletY) {
        long time = _robot.getTime();  // ADDED -1.. bug????
        double greatestDistance = Double.MAX_VALUE;
        Situation best = null;
        for (int i = _nodeQueue.size(); --i >= 0; ) {
            Situation s = _nodeQueue.get(i);
            double d = Math.abs(s.getDistance(time) - Point2D.distance(s.RX, s.RY, bulletX, bulletY));
            if (d < greatestDistance && d < 20d) {
                greatestDistance = d;
                best = s;
            }
        }
        return best;
    }

    // e.getBullet().getX()
    public Situation registerBulletHit(double bulletX, double bulletY) {
        Situation best = getInterceptingSituation(bulletX, bulletY);

        if (best != null) {
            best.registerHit(bulletX, bulletY);
        }

        return best;
    }

    public double getWallTries(double heading, double dir, double x, double y, double distance) {

        double wallIncrement = 0.0407d * dir;
        double eHeading = heading;
        double nextX = 0;
        double nextY = 0;
        double wallTries = -1;
        do {
            eHeading += wallIncrement;
            nextX = x + Math.sin(eHeading) * distance;
            nextY = y + Math.cos(eHeading) * distance;
            wallTries++;
        } while (_fieldRect.contains(nextX, nextY) && wallTries < 20);

        return wallTries;
    }


}
