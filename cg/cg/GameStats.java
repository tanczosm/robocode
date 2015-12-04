package cg;

import java.util.ArrayList;

public class GameStats {
    public int _hits = 0;
    public int _misses = 0;
    public double _distance = 0;
    public double totalEnemyDamage = 0;
    public double weightedEnemyHitrate = 0;
    public double weightedEnemyFirerate = 0;
    private double _totalDistance;

    public ArrayList<Integer> _lastHits = new ArrayList<Integer>();

    public void addShot (double bulletPower, double distance)
    {
        double botWidth = 2 * Math.atan(25 / (distance-18));
        double hitChance = botWidth / CTUtils.maxEscapeAngle(CTUtils.bulletVelocity(bulletPower));
        weightedEnemyFirerate += 1 / hitChance;
    }

    public void add(boolean hit, double bulletPower, double distance) {
        double botWidth = 2 * Math.atan(25 / (distance-18));
        double hitChance = botWidth / CTUtils.maxEscapeAngle(CTUtils.bulletVelocity(bulletPower));
        if (hit) {

            weightedEnemyHitrate += 1 / hitChance;
            totalEnemyDamage += CTUtils.bulletDamage(bulletPower);

            _hits++;
            _totalDistance += distance;
            _lastHits.add(1);
        } else {
            _misses++;
            _lastHits.add(0);
        }

        if (_lastHits.size() > 100)
        {
            _lastHits.remove(0);
        }
    }

    public boolean enableFlattener (double threshold)
    {
        return (weightedEnemyHitrate / weightedEnemyFirerate) > threshold;
    }

    public void addDamage(int distance) {

    }

    public double getAverageDistance() {
        return (double) _totalDistance / (double) (_hits);
    }

    public double getHitRatio() {
        double total = _hits + _misses;

        return (total == 0 ? 0 : _hits / total);
    }

    public double getRollingHitRatio(int n) {
        if (n > 100) n = 100;
        if (n > _lastHits.size())
            n = _lastHits.size();

        int count = 0;
        double total = 0;

        for (int i = 0; i < n; i++)
        {
            count++;
            total += _lastHits.get(i);
        }

        return (count == 0 ? 0 : total / count);
    }

    public int getTotal() {
        return _hits + _misses;
    }

    public void reset() {
        _hits = 0;
        _misses = 0;
    }
}
