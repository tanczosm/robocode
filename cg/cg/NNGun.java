package cg;

import robocode.AdvancedRobot;
import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.ml.train.MLTrain;
import org.encog.neural.freeform.FreeformNetwork;
import org.encog.neural.freeform.training.FreeformBackPropagation;
import org.encog.neural.freeform.training.FreeformResilientPropagation;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.neural.networks.training.propagation.back.Backpropagation;
import org.encog.neural.networks.training.propagation.resilient.ResilientPropagation;
import org.encog.util.Format;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Gun designed for neural network-based targeting
 */
public class NNGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    // Neural network stuff
    public static BasicNetwork basicNetwork;


    public static final int GF_ZERO = 30; // 23; //15;
    public static final int GF_ONE = 60; //46; //30;

    private List<NNBullet> waves = new ArrayList<NNBullet>();
    private NNBullet newWave = null;
    private Situation _lastSituation = null;
    int direction = 1;

    public NNGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;

        // create the basic network
        basicNetwork = new BasicNetwork();
        basicNetwork.addLayer(new BasicLayer(null, true, 39));
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 2));
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, 61));
        basicNetwork.getStructure().finalizeStructure();
        basicNetwork.reset();
        basicNetwork.reset(1000);


    }

    public String getName() {
        return "NN";
    }

    public void update() {

        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        // Let's process the waves now:
        for (int i = 0; i < waves.size(); i++) {
            NNBullet currentWave = (NNBullet) waves.get(i);

            if (currentWave.checkHit(ex, ey, _robot.getTime())) {

                // Train based on waves.returnSegment
                // waves.inputs
                // waves.outputs (2 double arrays for training)
                System.out.println("INP="+Arrays.toString(currentWave.inputs));
                System.out.println(Arrays.toString(currentWave.outputs));


                waves.remove(currentWave);
                i--;
            }

        }
    }

    public void takeVirtualShot(Situation s, double bearing) {
        super.takeVirtualShot(s, bearing);

        //newWave.startBearing = bearing; // Update bearing to actual bearing..


        if (newWave != null)
            waves.add(newWave);

    }

    public double [] getInputForSituation (Situation s)
    {

        /*
            Data	Range	Features
            Bullet flight time (BFT)	0 - 105	11
            Lateral velocity	0 - 8	11
            Lateral acceleration	-2 - +2	11
            Approach velocity	-8 - +8	9
            Dist. traveled last 10 ticks	0 - 65	6
            Ticks since velocity change	0 - BFT	7
            Ticks since direction change	0 - BFT	7
            Forward radians to wall	0 - 1.5	7
            Reverse radians to wall	0 - 1.0	4
            Current guess factor	-1 - +1	11
         */
        /*
            LateralVelocity
            Acceleration
            NormalizedDistance
            WallTriesForward
            WallTriesBack
            AdvancingVelocity
        */

        // Distance - Range 0 - 800, split into 11 features
        double[] fdistance = RBFUtils.processDataIntoFeatures(Math.min(s.Distance, 800), 800, RBFUtils.getCenters(0, 800, 11));

        // Lateral Velocity - Range 0 - 8, split into 8 features
        double[] flatvel = RBFUtils.processDataIntoFeatures(s.LateralVelocity*8.0, 8.0, RBFUtils.getCenters(0, 8, 8));

        // Acceleration - Range 0 - 1.0, split into 11 features
        double[] faccel = RBFUtils.processDataIntoFeatures(s.Acceleration, 1.0, RBFUtils.getCenters(0, 1.0, 11));

        // Advancing Velocity - Range -8.0 - 8.0, split into 9 features
        double advancingVelocity = s.AdvancingVelocity * 16d; // +/- 8.0
        double[] fadvancevel = RBFUtils.processDataIntoFeatures(advancingVelocity, 16.0, RBFUtils.getCenters(-8d, +8d, 9));

        // Need distance delta
        // Need SinceVelocityChange

        // Wall Tries Forward - Range 0.0 - 20.0, split into 7 features
        double[] ffwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesForward*20, 20.0, RBFUtils.getCenters(0, 20, 7));

        // Wall Tries Backward - Range 0.0 - 20.0, split into 4 features
        double[] fbwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesBack*20, 20.0, RBFUtils.getCenters(0, 20, 4));

        return RBFUtils.mergeFeatures(fdistance, flatvel, faccel, fadvancevel, ffwalltries, fbwalltries);

        // Situation s already contains normalized data
        /*
            public double EnemyHeading;
            public double Bearing;
            public double BearingRadians;
            public double NormalizedDistance;
            public double Distance;
            public double DistanceDelta;
            public double PlayerLateralVelocity;
            public double LateralVelocity;
            public double AdvancingVelocity;
            public double Acceleration;
            public double Velocity;
            public double SinceVelocityChange;
            public double WallTriesForward;
            public double WallTriesBack;
            public double Direction;
            public double BulletVelocity;
            public double MaxAngle;
            public double GuessFactor;
        */
    }

    public void addSituation(Situation s) {
        // Do nothing
        _lastSituation = s;
    }

    public double projectBearing(Situation s, double x, double y, double enemyHeading) {

        if (s == null)
            return 0;

        //double[] currentStats = getStats(s); //stats[(int)(s.Distance / 100)];
        double absBearing = enemyHeading; //_robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // don't try to figure out the direction they're moving
        // they're not moving, just use the direction we had before
        if (_radarScanner.nme.velocity != 0) {
            if (Math.sin(_radarScanner.nme.headingRadians - absBearing) * _radarScanner.nme.velocity < 0)
                direction = -1;
            else
                direction = 1;
        }
        //w.bearingDirection = (double)direction *Math.asin(8D/newWave.getBulletSpeed())/GF_ZERO;

        newWave = new NNBullet(x, y, absBearing, ((Double) _radarScanner._surfAbsBearings.get(0)).doubleValue(), _radarScanner.FIRE_POWER,
                direction, _robot.getTime(), getInputForSituation(s));


        int bestGF = GF_ZERO;    // initialize it to be in the middle, guessfactor 0.

        // TODO: USE NEURAL NET TO FIND BETTER GF

        newWave.fireIndex = bestGF;

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double) (bestGF - GF_ZERO) / (double) GF_ZERO;

        double angleOffset = direction < 0 ? -(guessfactor * newWave.lowGF * newWave.maxEscapeAngle) :
                (guessfactor * newWave.highGF * newWave.maxEscapeAngle);

        return angleOffset;


    }
}
