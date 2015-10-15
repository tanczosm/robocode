package cg;

public class GameStats {
    public int _hits = 0;
    public int _misses = 0;
    public int _distance = 0;
    private double _totalDistance;

    public void add(boolean hit, int distance) {
        if (hit) {
            _hits++;
            _totalDistance += distance;
        } else
            _misses++;

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

    public int getTotal() {
        return _hits + _misses;
    }

    public void reset() {
        _hits = 0;
        _misses = 0;
    }
}
