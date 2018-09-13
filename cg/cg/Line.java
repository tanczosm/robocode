package cg;


import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class Line extends Line2D.Double {
    private static final long serialVersionUID = 6192542150615359687L;

    public Line() {}

    public Line(double x1, double y1, double x2, double y2) {
        super(x1, y1, x2, y2);
    }

    public Line(double[] a, Point2D.Double b) {
        super(a[0],a[1],b.x,b.y);
    }
    public Line(Point2D.Double a, Point2D.Double b) {
        super(a.x,a.y,b.x,b.y);
    }

    public static final Line projection(double x, double y, double angle, double dist) {
        Line line = new Line();
        line.x1 = x;
        line.y1 = y;
        line.x2 = x + Math.sin(angle) * dist;
        line.y2 = y + Math.cos(angle) * dist;
        return line;
    }

    public static final Line projection(double x, double y, double angle, double dist1, double dist2) {
        Line line = new Line();
        line.x1 = x + Math.sin(angle) * dist1;
        line.y1 = y + Math.cos(angle) * dist1;
        line.x2 = x + Math.sin(angle) * dist2;
        line.y2 = y + Math.cos(angle) * dist2;
        return line;
    }

    public Point2D.Double getMidPoint() {
        return new Point2D.Double((x1+x2)/2.0,(y1+y2)/2.0);
    }

    public double lengthSq() {
        return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);
    }
}