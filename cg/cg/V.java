package cg;

import robocode.*;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class V extends AdvancedRobot {

    public enum Modes
    {
        UNRESTRICTED,
        SURF_ONLY,
        MOVEMENT_CHALLENGE, // Surf but use Raiko gun
        TARGET_ONLY
    }

    public static StatsTracker playerStats = new StatsTracker();
    public static RadarScanner _radarScanner = new RadarScanner();
    public static Targeting _targeting = null;
    public static Movement _movement = null;

    public static Modes mode = Modes.UNRESTRICTED; // Modes.MOVEMENT_CHALLENGE;
    public static RaikoGun _raikoGun;

    public void run() {
        _radarScanner.setRobot(this);
        _raikoGun = new RaikoGun(this);

        if (_targeting == null && (mode != Modes.SURF_ONLY && mode != Modes.MOVEMENT_CHALLENGE)) {
            _targeting = new Targeting(this, _radarScanner, playerStats);
        }

        if (_movement == null && mode != Modes.TARGET_ONLY)
            _movement = new Movement(this, _radarScanner);

        setBulletColor(new Color(255, 248, 96));
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        //setAllColors(Color.black);
        turnRadarRightRadians(Double.POSITIVE_INFINITY);

        do {
            if (mode != mode.SURF_ONLY && mode != Modes.MOVEMENT_CHALLENGE) {
                _targeting.selectFiringPower(_radarScanner.nme.distance);
                _targeting.process(null);
            }
            if (mode == Modes.MOVEMENT_CHALLENGE)
            {
                _raikoGun.run();
            }

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

        if (_targeting != null)
            _targeting.onBattleEnded();

        if (_movement != null)
            _movement.onBattleEnded();
    }

    @Override
    public void onRoundEnded(RoundEndedEvent event)
    {

        if (_targeting != null)
            _targeting.onRoundEnded();

        if (_movement != null)
            _movement.onRoundEnded();

    }

    public void onScannedRobot(ScannedRobotEvent e) {


        if (_radarScanner._lastScan == null) {
            _radarScanner._lastScan = e;
            return;
        }

        if (mode == Modes.MOVEMENT_CHALLENGE)
        {
            _raikoGun.onScannedRobot(e);
        }

        //double enemyHeading = getHeadingRadians() + _radarScanner.nme.bearingRadians;
        Situation s = _radarScanner.processScanEvent(e, true);

        if (_movement != null)
            _movement.scan(e);


        _radarScanner._oppEnergy = _radarScanner._lastScan.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _radarScanner.nme.location = CTUtils.project(_radarScanner._myLocation, (Double) _radarScanner._surfAbsBearings.get(0) - Math.PI, e.getDistance());

        if (_movement != null)
            _movement.update(e);

        if (_targeting != null) {
            _targeting.selectFiringPower(_radarScanner.nme.distance);
            _targeting.process(s);
        }

        _radarScanner.postProcessScanEvent(e);
    }


}