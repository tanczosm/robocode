package ja;

import ja.ags.utils.dataStructures.trees.thirdGenKD.DistanceFunction;
import ja.ags.utils.dataStructures.trees.thirdGenKD.KdTree;
import ja.ags.utils.dataStructures.trees.thirdGenKD.NearestNeighborIterator;
import robocode.AdvancedRobot;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import static ja.CTUtils.*;

public class KNNGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    public double enemyProjectedDistance;
    public ArrayList<Point2D.Double> targetPoints; // just for storing targeting data


    public static int MAX_SELECTED_SITUATIONS = 12;
    public static double KERNEL_DENSITY_BANDWIDTH = 14d;  // defaults to 6.0d

    public DistanceFunction distanceFunction = new ManhattanDistanceFunction();
    //									LateralVelocity, Acceleration, NormalizedDistance, WallTriesForward, WallTriesBack, AdvancingVelocity
    public double[] SITUATION_WEIGHTS = {0.5, 0.0, 5, 5, 0.5, 0.5};



    public KNNGun(AdvancedRobot robot, RadarScanner radarScanner)
    {
        _robot = robot;
        _radarScanner = radarScanner;

        targetPoints = new ArrayList<Point2D.Double>();
    }

    public void addSituation(Situation s)
    {

    }

    public String getName()
    {
        return "KNN";
    }

    public void update()
    {

    }

    public double projectBearing(Situation scan, double x, double y, double enemyHeading)
    {
        if (scan == null)
            return 0;

        double gunHeading = _robot.getGunHeadingRadians();
        double finalBearing = 0;

        MAX_SELECTED_SITUATIONS = Math.max(1, Math.min(225, _radarScanner.situations.size() / 9));

        NearestNeighborIterator<Situation> similarSituationsIterator = _radarScanner.situations.getNearestNeighborIterator(scan.getPoint(), MAX_SELECTED_SITUATIONS, distanceFunction);
        ArrayList<Situation> bestNodes = new ArrayList<Situation>();

        int ow = 10;
        Graphics2D g = _robot.getGraphics();

        //	setMaxVelocity(500);
        g.setColor(new Color(0xff, 0x00, 0x00, 0x80));
        while (similarSituationsIterator.hasNext()) {
            Situation node = similarSituationsIterator.next();

            bestNodes.add(node);

            if (CrushTurtle.PaintPossibleSituations) {
                double tbear = (node.BearingRadians * scan.MaxAngle) / node.MaxAngle;
                Point2D.Double outp = project(_radarScanner._myLocation, enemyHeading + tbear, node.Distance + node.DistanceDelta);

                //	if (Math.abs(node.Distance-distance) < 150)
                g.drawOval((int) (outp.getX() - ow / 2), (int) (outp.getY() - ow / 2), ow, ow);
            }
        }


        int results = bestNodes.size();
        int bestIndex = 0;
        double bestDensity = 0;

        for (int i = results; --i >= 0; ) {

            double density = 0;
            double u;
            for (int j = results; --j >= 0; ) {
                double kdeDiff = bestNodes.get(i).Bearing - bestNodes.get(j).Bearing;
                //double kdeDiff = Math.abs(bestNodes.get(i).Distance - bestNodes.get(j).Distance);

                density += Math.exp(
                        (
                                u = (kdeDiff) / KERNEL_DENSITY_BANDWIDTH // BAND_WIDTH
                        )
                                * u
                                * -0.5d
                );
            }


            if (density > bestDensity) {
                bestDensity = density;
                bestIndex = i;
            }
        }

        if (results > 0) {

            Situation best = bestNodes.get(bestIndex);
            // Chalk style
            double theBearing = (best.BearingRadians * scan.MaxAngle) / best.MaxAngle;
            //double theBearing = (scan.Direction * best.GuessFactor) * scan.MaxAngle;

            // GF targeting style
			/*
			double angleOffset = scan.Direction * best.GuessFactor * scan.MaxAngle;
            double theBearing = Utils.normalRelativeAngle(absBearing + angleOffset - gunHeading);
			*/
            enemyProjectedDistance = best.Distance + best.DistanceDelta;
            Point2D.Double _nextLocation = project(_radarScanner._myLocation, _robot.getHeadingRadians(), _robot.getVelocity()); // BUG? _robot.getVelocity() was negative
            Point2D.Double target = project(_nextLocation, Utils.normalRelativeAngle(enemyHeading + theBearing), enemyProjectedDistance);

            if (_robot.getGunHeat() == 0) {
                targetPoints.add(target);
                if (targetPoints.size() > 8)
                    targetPoints.remove(0);

            }

            double maxAngle = CTUtils.maxEscapeAngle(CTUtils.bulletVelocity(_radarScanner.FIRE_POWER));
            finalBearing = Utils.normalRelativeAngle(theBearing);
            if (!RadarScanner._fieldRect.contains(target) || finalBearing != CTUtils.clamp(finalBearing, -maxAngle, maxAngle))
            {
                finalBearing = Double.MAX_VALUE;
            }
			/*
			double mea = maxEscapeAngle(bulletVelocity(FIRE_SPEED)); // * (180/Math.PI);
		 	double ang = enemyHeading + absoluteBearing(_myLocation, target) - gunHeading;
			*/

  //          tempRotate = tr;

			/*
			if (Math.abs(ang) <= Math.abs(mea))
			{
				//System.out.println("Shot guaranteed to miss [ang=" + ang + ", mea=" + mea + "]");
				fireReady = true;

			}
			*/

			/*
			if (Math.abs(theBearing) <= mea)
				fireReady = true;
			else
				fireReady = false;
			*/


			/*
			double projectedDistance = best.Distance + best.DistanceDelta;
			double projectedX = x + Math.sin(enemyHeading + best.BearingRadians) * projectedDistance;
			double projectedY = y + Math.cos(enemyHeading + best.BearingRadians) * projectedDistance;
			if(_fieldRect.contains(projectedX, projectedY)){
				tempRotate = Utils.normalRelativeAngle(enemyHeading + theBearing - gunHeading);
			}
			else
			{
				tempRotate = Double.MAX_VALUE;
			}
			*/
        }

        double intensity = 0;
        int count = 0;
        for (Point2D.Double target : targetPoints) {
            intensity++;
            double it = intensity / targetPoints.size();

            g.setColor(new Color(0xff, 0xff, 0x00, (int) (it * 255)));

            if (count == targetPoints.size()-1)
                g.setColor(new Color(0xff, 0x00, 0x00, 0xff));

            g.drawOval((int) (target.getX() - ow / 2), (int) (target.getY() - ow / 2), ow, ow);
            count++;
        }

        return finalBearing;
    }


    public class ManhattanDistanceFunction implements DistanceFunction {
        @Override
        public double distance(double[] p1, double[] p2) {
            double d = 0;

            for (int i = 0; i < p1.length; i++) {

                double diff = Math.abs(p1[i] - p2[i]);

                if ((diff > 100.0 / 800d) && i == 3) // normalized distance
                    d += 100000; // poison it
                else

                    d += diff * SITUATION_WEIGHTS[i];

            }

            return d;
        }

        @Override
        public double distanceToRect(double[] point, double[] min, double[] max) {
            double d = 0;

            for (int i = 0; i < point.length; i++) {
                double diff = 0;
                if (point[i] > max[i]) {
                    diff = Math.abs(point[i] - max[i]);
                } else if (point[i] < min[i]) {
                    diff = Math.abs(point[i] - min[i]);
                }
                d += diff * SITUATION_WEIGHTS[i];
            }

            return d;
        }
    }
}
