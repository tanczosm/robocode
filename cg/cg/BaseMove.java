package cg;

import robocode.*;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;


/**
 * Created by tanczosm on 10/14/2015.
 */
public abstract class BaseMove {

    abstract void scan(ScannedRobotEvent e);
    abstract void update(ScannedRobotEvent e);
    abstract void onHitByBullet(HitByBulletEvent e);
    abstract void onBulletHitBullet(BulletHitBulletEvent e);
    abstract String getName();
    abstract void onBulletHit(BulletHitEvent e);
    abstract public void onBulletMissed(BulletMissedEvent e);
    abstract public void onBattleEnded();
    abstract public void onRoundStarted();
    abstract public void onRoundEnded();

}
