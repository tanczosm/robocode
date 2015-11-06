package cg;

import robocode.AdvancedRobot;
import robocode.BattleRules;
import robocode.Bullet;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;

public class Targeting {

    public static Bullet lastBullet = null;

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;
    private StatsTracker _playerStats;

    public ArrayList<BaseGun> guns;
    public BaseGun _currentGun;

    public boolean _aiming = false;
    public int _shotsFired = 0;
    public int _lastShotsFired = 0;
    public double _lastBearing = Double.POSITIVE_INFINITY;
    public Situation _lastSituation = null;

    public BattleRules _battleRules;
    public double _coolingRate;

    public int maxGunSwitches = 1;
    public int totalSwitches = 0;


    public Targeting(AdvancedRobot robot, RadarScanner radarScanner, StatsTracker playerStats) {
        this._robot = robot;
        this._radarScanner = radarScanner;
        this._playerStats = playerStats;

        _coolingRate = _robot.getGunCoolingRate();

        // Create all our guns here
        guns = new ArrayList<BaseGun>();

        guns.add(new HotGun(_robot, _radarScanner));
        guns.add(new GFGun(_robot, _radarScanner));
        guns.add(new NNGun(_robot, _radarScanner));

        // Next let's make sure the radar scanner knows to update all of our guns
        for (BaseGun gun : guns) {
            this._radarScanner.registerGun(gun);
        }

        _currentGun = guns.get(2);
    }

    public void setBattleRules(BattleRules battleRules) {
        this._battleRules = battleRules;
    }

    public void onBattleEnded() {
        for (BaseGun gun : guns) {
            gun.onBattleEnded();
        }
    }

    public void onRoundEnded()
    {
        for (BaseGun gun : guns) {
            gun.onRoundEnded();
        }

    }

    public void selectGun() {

        if (_radarScanner.nme.energy < 0.1) {
            _currentGun = guns.get(0);
        }
        else {
            _currentGun = guns.get(2);
        }

    }

    public void process(Situation s) {
        if (_lastSituation == null) {
            _lastSituation = s;

            return; // We need two good scans in a row to get things moving correctly
        }

        double x = _robot.getX(); // (s == null ? _robot.getX() : s.RX);
        double y = _robot.getY(); //(s == null ? _robot.getY() : s.RY); //_robot.getY();
        double smallOffset = 0; //(Math.PI/180/16); //(Math.PI/180/4);
        double distance = (_radarScanner._lastScan == null ? 0 : _radarScanner._lastScan.getDistance());
        double enemyHeading = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians; // smallOffset + (_radarScanner._lastScan == null ? 0 : _radarScanner.nme.bearingRadians);
        double gunHeading = _robot.getGunHeadingRadians();

        MovSimStat next = CTUtils.nextLocationFull(_robot);
        enemyHeading = next.h + _radarScanner.nme.bearingRadians;

        //System.out.println("Old enemyHeading: " + (_robot.getHeadingRadians() + _radarScanner.nme.bearingRadians) + ", New heading: " + enemyHeading);

        //double bearing = _lastBearing; //(s == null ? (_lastBearing == Double.POSITIVE_INFINITY ? _currentGun.projectBearing(_lastSituation, x, y, enemyHeading) : _lastBearing) : _currentGun.projectBearing(s, x, y, enemyHeading));

        if (s != null)
            _lastSituation = s;

        double bearing = !_aiming ? _currentGun.projectBearing(s, next.x, next.y, enemyHeading) : _lastBearing;
        _lastBearing = bearing;


        Point2D.Double target = CTUtils.project(_radarScanner._myLocation, Utils.normalRelativeAngle(enemyHeading + bearing), 1000);
        Graphics2D g = _robot.getGraphics();
        g.drawLine((int) _robot.getX(), (int) _robot.getY(), (int) target.getX(), (int) target.getY());

        /*
        double gunTurnRate = Rules.GUN_TURN_RATE_RADIANS;
        double turnRemaining = _robot.getGunTurnRemainingRadians();
        double gunTicks = turnRemaining / gunTurnRate;
        double bearingTicks = bearing / gunTurnRate;
        */
        //System.out.println("Gun turn remaining " + turnRemaining + ", gunTurnRate: " + gunTurnRate + ", bearing ticks: " + bearingTicks);

        update();
        //double bearing = _currentGun.projectBearing(s, x, y, enemyHeading);

        selectGun();

        if (RadarScanner.FIRE_POWER > 0 && _robot.getGunHeat() / _coolingRate < 2d) {

            // Rotate gun according to bearing
            if (bearing < Double.MAX_VALUE && _robot.getGunHeat() <= 2*_coolingRate/* && distance > 70d*/) {
                _robot.setTurnGunRightRadians(Utils.normalRelativeAngle(enemyHeading - gunHeading + bearing));
            }

            // Check to see if gun finished rotating and we can fire
            _aiming = true;
            if (bearing < Double.MAX_VALUE
                    && (Math.abs(_robot.getGunTurnRemainingRadians()) < Math.atan(8d / _radarScanner.nme.distance) || distance < 70d)
                    ) {
                //s.IsRealBullet = true;

                //Bullet b;

                if ((_robot.getEnergy() > (RadarScanner.FIRE_POWER + 0.01) || _radarScanner.nme.energy == 0 || V.mode == V.Modes.TARGET_ONLY) && (lastBullet = _robot.setFireBullet(RadarScanner.FIRE_POWER)) != null) {
                    _aiming = false;
                    _shotsFired++;
                    reportAccuracy();
                    //                bearing = b.getHeadingRadians();
                    //                System.out.println("bearing: " + bearing + ", enemyHeading: " + enemyHeading + ", Bullet Heading: " + b.getHeadingRadians());

                    for (BaseGun gun : guns) {
                        if (gun != _currentGun) {
                            double altBearing = gun.projectBearing(s, next.x, next.y, enemyHeading);
                            gun.takeVirtualShot(_lastSituation, altBearing);
                        } else {
                            double rotationError = (enemyHeading - _robot.getGunTurnRemainingRadians());
                            rotationError = 0;
                            gun.takeVirtualShot(_lastSituation, bearing + rotationError);

                            //System.out.println("Rotation error: " + rotationError);

                        }
                    }
                }


            }

        }

        if (!_aiming) {
            _robot.setTurnGunRightRadians(Utils.normalRelativeAngle(enemyHeading - gunHeading));
        }


    }

    public void CheckBullets() {
        for (BaseGun gun : guns) {
            gun.checkVirtualBullets(_robot.getTime(), _radarScanner.nme.enemyBox);
        }
    }

    public void update() {
        for (BaseGun gun : guns) {
            gun.update();
        }
    }

    public void reportAccuracy() {
        NumberFormat formatter = new DecimalFormat("#0.00");
        /*
        System.out.print("Current Accuracy: ");
        for (BaseGun gun : guns) {
            System.out.print("[" + gun.getName() + "=" + formatter.format(gun.getRatingPercent() * 100) + "]" + (_currentGun == gun ? "* " : " "));
        }
        System.out.println();
        */
    }

    public void selectFiringPower(double distance) {


        double playerEnergy = _robot.getEnergy();

        double enemyBulletPower = _radarScanner.nme.lastBulletPower;
        double _shotsTotal = _playerStats.getSegmentedStat((int) distance).getTotal();
        double _shotAccuracy = _playerStats.getSegmentedStat((int) distance).getHitRatio() * 100;
        double _enemyShotsTotal = _radarScanner.nme.enemyShotHits + _radarScanner.nme.enemyShotMisses;
        double _enemyShotAccuracy = (_enemyShotsTotal == 0 ? 0 : ((_radarScanner.nme.enemyShotHits / (_enemyShotsTotal)) * 100));


        RadarScanner.FIRE_POWER = 1.92d; //1.72d; //2.4d; //1.95;

        if (_radarScanner.nme.distance < 70d)
            RadarScanner.FIRE_POWER = 3;
        if (_radarScanner.nme.distance < 100d || _shotAccuracy > 32d)
            RadarScanner.FIRE_POWER = 2.95d;
        else {
            //power = scale based on my energy and enemy energy expected at the time this bullet hits if they continue to shoot every time gunheat is 0
            int ticks = (int) (_radarScanner.nme.distance / (20 - RadarScanner.FIRE_POWER * 3));  // How many ticks will it take to traverse this distance
            int expectedTicksPerShot = (int) ((1 + (RadarScanner.FIRE_POWER / 5)) / 0.1d);
            double playerExpectedEnergy = (playerEnergy - RadarScanner.FIRE_POWER);
            int enemyExpectedTicksPerShot = (int) ((1 + (enemyBulletPower / 5)) / 0.1d);  // Enemy gunheat / 0.1 .. Gives how many ticks per shot
            double enemyExpectedEnergy = _radarScanner.nme.energy - (ticks / enemyExpectedTicksPerShot) * enemyBulletPower;  // Assume they stick with same bullet power

            if (enemyBulletPower < RadarScanner.FIRE_POWER && (_enemyShotAccuracy > _shotAccuracy))
                RadarScanner.FIRE_POWER = Math.max(0.1, enemyBulletPower - 0.1);
        }


        RadarScanner.FIRE_POWER = 1.92d;

        if (RadarScanner.FIRE_POWER > playerEnergy || playerEnergy < 15) {
            RadarScanner.FIRE_POWER = 0.1d;
        }
        /*
        if (_robot.getRoundNum() < 3)
            RadarScanner.FIRE_POWER = 0.1d; // Use dodging capability to collect firing data.. we should live longer and get smarter
        */

        if (V.mode == V.Modes.TARGET_ONLY)
            RadarScanner.FIRE_POWER = 3;


        RadarScanner.FIRE_SPEED = 20 - RadarScanner.FIRE_POWER * 3;

    }
}
