package cg;


public class MovSimStat {
    public double x;
    public double y;
    public double v;
    public double h;
    public double w;

    public double danger;
    public int tickDistance;
    public int direction; // left or right for now

    public MovSimStat(double x, double y, double v, double h, double w) {
        this.x = x;
        this.y = y;
        this.v = v;
        this.h = h;
        this.w = w;
    }


}