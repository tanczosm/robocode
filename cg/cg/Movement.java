package cg;

import robocode.*;

import java.util.ArrayList;

/**
 * Created by tanczosm on 10/14/2015.
 */
public class Movement {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;
    private ArrayList<BaseMove> movers;
    private BaseMove selectedMover;

    public Movement (AdvancedRobot robot, RadarScanner radarScanner)
    {
        this._robot = robot;
        this._radarScanner = radarScanner;
        this.movers = new ArrayList<BaseMove>();

        this.movers.add(new GTSurferMove(robot, radarScanner));
        this.movers.add(new CTSurferMove(robot, radarScanner));

        selectedMover = this.movers.get(1);
    }

    public void selectMover (String name)
    {
        int index = 0;

        for (int i = 0; i < movers.size(); i++)
        {
            BaseMove bm = movers.get(i);

            if (bm != null)
            {
                if (bm.getName().equals(name))
                {
                    index = i;
                    break;
                }
            }
        }

        selectedMover = movers.get(index);
    }

    public void update(ScannedRobotEvent e)
    {
        selectedMover.update(e);
    }

    /* Call mover events */
    public void onBulletHit(BulletHitEvent e) {
        RadarScanner._oppEnergy -= e.getEnergy();

        selectedMover.onBulletHit(e);
    }

    public void onHitByBullet(HitByBulletEvent e) {

        RadarScanner._oppEnergy += robocode.Rules.getBulletHitBonus(e.getBullet().getPower());

        selectedMover.onHitByBullet(e);
    }

    public void onBulletHitBullet(BulletHitBulletEvent e)
    {
        selectedMover.onBulletHitBullet(e);
    }

    public void onBulletMissed(BulletMissedEvent e)
    {
        selectedMover.onBulletMissed(e);
    }
}
