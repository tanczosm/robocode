package cg;

import robocode.*;

import java.awt.geom.Point2D;
import java.util.ArrayList;

public class V extends AdvancedRobot {

    private boolean turnRadar = true;
    public static StatsTracker playerStats = new StatsTracker();
    public static RadarScanner _radarScanner = new RadarScanner();
    public static Targeting _targeting = null;
    public static Movement _movement = null;
    public ArrayList<EnemyWave> _enemyWaves;


    static final int GF_ZERO = 23; //23;
    static final int GF_ONE = 46; // 46;
    public static int BINS = GF_ONE + 1;
    private static double _surfStats[][][] = new double[6][5][BINS];

    public void run() {
        _enemyWaves = new ArrayList<EnemyWave>();
        _radarScanner.setRobot(this);
        _radarScanner.nme.waves = _enemyWaves;

        Enemy.waves = _enemyWaves;

        if (_targeting == null) {
            _targeting = new Targeting(this, _radarScanner, playerStats);
        }

        if (_movement == null)
            _movement = new Movement(this, _radarScanner);

        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        //setAllColors(Color.black);
        turnRadarRightRadians(Double.POSITIVE_INFINITY);

        do {
            _targeting.selectFiringPower(_radarScanner.nme.distance);
            _targeting.process(null);

            scan();

        } while (true);
    }

    public void onHitByBullet(HitByBulletEvent e) {

        _radarScanner.nme.enemyShotHits++;

        if (_movement != null) _movement.onHitByBullet(e);
    }

    public void onBulletHitBullet(BulletHitBulletEvent e) {

        Bullet b = e.getBullet();
        _radarScanner.registerBulletHit(b.getX(), b.getY());

        if (_movement != null) _movement.onBulletHitBullet(e);
    }

    public void onBulletHit(BulletHitEvent e) {
        if (_movement != null) _movement.onBulletHit(e);

        Situation best = _radarScanner.registerBulletHit(e.getBullet().getX(), e.getBullet().getY());

        if (best != null)
            playerStats.add(true, (int) best.Distance);
        else
            playerStats.add(true, (int) _radarScanner.nme.distance);

    }

    public void onBulletMissed(BulletMissedEvent e) {

        Situation best = _radarScanner.getInterceptingSituation(e.getBullet().getX(), e.getBullet().getY());

        if (best != null)
            playerStats.add(false, (int) best.Distance);
        else
            playerStats.add(false, (int) _radarScanner.nme.distance);

        if (_movement != null) _movement.onBulletMissed(e);
    }

    @Override
    public void onBattleEnded(BattleEndedEvent event) {
        System.out.println("Battle ended..");

        _targeting.onBattleEnded();
    }

    public void onScannedRobot(ScannedRobotEvent e) {


        if (_radarScanner._lastScan == null) {
            _radarScanner._lastScan = e;
            return;
        }

        double enemyHeading = getHeadingRadians() + _radarScanner.nme.bearingRadians;
        Situation s = _radarScanner.processScanEvent(e, true);

        try {
            double bulletPower = _radarScanner._oppEnergy - e.getEnergy();
            if (bulletPower < 3.01 && (bulletPower > 0.09 || (bulletPower > 0 && (s.WallTriesForward > 5 && s.WallTriesBack > 5))) && _radarScanner._surfDirections.size() > 2) {
                EnemyWave ew = new EnemyWave();
                ew.fireTime = getTime() - 1;
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
//(int)((s.WallTriesBack+s.WallTriesForward)) (0..2)
                _radarScanner.nme.lastBulletPower = bulletPower;
            }


            if (bulletPower >= 0 && bulletPower < 3.01)
                _radarScanner.nme.lastShotTime = getTime();

        } catch (ArrayIndexOutOfBoundsException em) {
            //System.out.println(em.getMessage());
            System.out.println();
        }


        _radarScanner._oppEnergy = _radarScanner._lastScan.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _radarScanner.nme.location = CTUtils.project(_radarScanner._myLocation, (Double) _radarScanner._surfAbsBearings.get(0) - Math.PI, e.getDistance());

        updateWaves();

        _movement.update(e);

        _targeting.selectFiringPower(_radarScanner.nme.distance);
        _targeting.process(s);

        _radarScanner.postProcessScanEvent(e);
    }

    public void updateWaves() {
        for (int x = 0; x < _enemyWaves.size(); x++) {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

            ew.distanceTraveled = (getTime() - ew.fireTime) * ew.bulletVelocity;
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

}