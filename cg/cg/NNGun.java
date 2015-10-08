package cg;

import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.basic.BasicMLDataPair;
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

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

/**
 * Gun designed for neural network-based targeting
 */
public class NNGun extends BaseGun {

    private AdvancedRobot _robot;
    private RadarScanner _radarScanner;

    // Neural network stuff
    public BasicNetwork basicNetwork;
    public Backpropagation basicTrain;

    public BasicNetwork randomNetwork;
    public Backpropagation randomTrain;

    public static final int INPUT_LENGTH = 63;
    public static final int OUTPUT_LENGTH = 61;

    public static final int GF_ZERO = 30; // 23; //15;
    public static final int GF_ONE = 60; //46; //30;

    private List<NNBullet> waves = new ArrayList<NNBullet>();
    private NNBullet newWave = null;
    private Situation _lastSituation = null;
    int direction = 1;
    double currentGuessFactor = 0;

    private ArrayList<double[]> _hitQueueInputs;
    private ArrayList<double[]> _hitQueueOutputs;

    private ArrayList<MLDataPair> _theData;
    private ArrayList<MLDataPair> _randomData;
    private ArrayList<MLDataPair> _randomDataBuffer;

    private Random _rand;

    public NNGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;
        _hitQueueInputs = new ArrayList<double[]>();
        _hitQueueOutputs = new ArrayList<double[]>();
        _theData = new ArrayList<MLDataPair>();
        _randomData = new ArrayList<MLDataPair>();
        _randomDataBuffer = new ArrayList<MLDataPair>();

        _rand = new Random();

        // create the basic network
        basicNetwork = new BasicNetwork();
        basicNetwork.addLayer(new BasicLayer(null, true, INPUT_LENGTH));
        //basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 39));
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, OUTPUT_LENGTH));
        basicNetwork.getStructure().finalizeStructure();
        basicNetwork.reset();
        basicNetwork.reset(1000);

        // create the basic network
        randomNetwork = new BasicNetwork();
        randomNetwork.addLayer(new BasicLayer(null, true, INPUT_LENGTH));
        //basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 39));
        randomNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, OUTPUT_LENGTH));
        randomNetwork.getStructure().finalizeStructure();
        randomNetwork.reset();
        randomNetwork.reset(1000);

        _theData.add(new BasicMLDataPair(new BasicMLData(new double[INPUT_LENGTH]), new BasicMLData(new double[OUTPUT_LENGTH])));
        _randomData.add(new BasicMLDataPair(new BasicMLData(new double[INPUT_LENGTH]), new BasicMLData(new double[OUTPUT_LENGTH])));

        // create training data
        MLDataSet trainingSet = new BasicMLDataSet(_theData);
        MLDataSet randomTrainingSet = new BasicMLDataSet(_randomData);
//         MLDataSet trainingSet = new BasicMLDataSet(new double[1][INPUT_LENGTH], new double[1][OUTPUT_LENGTH]);

        // create two trainers
        basicTrain = new Backpropagation(basicNetwork, trainingSet, 0.7, 0.3);
        randomTrain = new Backpropagation(basicNetwork, randomTrainingSet, 0.7, 0.3);

        basicTrain.setBatchSize(1);
        randomTrain.setBatchSize(1);
    }

    public String getName() {
        return "NN";
    }

    public RobotState[] calculateEnemyMoves(boolean clockwise)  // May want to make the absBearing to be perp to player always
    {
        double maxVelocity = 8;
        double heading = _radarScanner.nme.headingRadians;
        double velocity = _radarScanner.nme.velocity;
        double absBearingRadians = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;
        double x = _radarScanner.nme.location.getX();
        double y = _radarScanner.nme.location.getY();
        double attackAngle = 0; // Assume the most field coverage?
        int currentTime = (int) _robot.getTime();

        /*
        boolean clockwise = false;

        // they're not moving, just use the direction we had before
        if ( _radarScanner.nme.velocity != 0)
        {
            if (Math.sin(_radarScanner.nme.headingRadians - absBearingRadians)*_radarScanner.nme.velocity < 0)
                clockwise = false;
            else
                clockwise = true;
        }
        */

        double maxEscapeAngle = CTUtils.maxEscapeAngle(_radarScanner.FIRE_SPEED);

        int possibleTicks = CTUtils.bulletTicks(_radarScanner.nme.distance + 32, _radarScanner.FIRE_POWER);
        int movesToFind = possibleTicks; //(int)CTUtils.clamp(3*possibleTicks, 20, 50); // Presently 3 and 50 are optimal

        Graphics2D g = _robot.getGraphics();
        RobotState[] moves = new RobotState[movesToFind];

        for (int i = 0; i < movesToFind; i++) {
            RobotState next = CTUtils.nextPerpendicularWallSmoothedLocation(
                    new Point2D.Double(x, y), absBearingRadians, velocity, maxVelocity, heading,
                    attackAngle, clockwise, currentTime,
                    _radarScanner._fieldRect, 800, 600,
                    140, false);

            moves[i] = next;
            x = next.location.getX();
            y = next.location.getY();
            velocity = next.velocity;
            heading = next.heading;
            absBearingRadians = CTUtils.absoluteBearing(_radarScanner._myLocation, next.location); // Location is probably relative to last shot

            g.setColor(new Color(0.36078432f, 0.56078434f, 1.0f));
            g.drawOval((int) x, (int) y, 2, 2);
        }

        return moves;
    }

    public RobotState[] getEnemyMoves() {
        RobotState[] right = calculateEnemyMoves(true);
        RobotState[] left = calculateEnemyMoves(false);

        RobotState[] moves = new RobotState[left.length + right.length];
        int middleIndex = left.length;

        int j = 0;
        for (int i = left.length - 1; i >= 0; i--) {
            moves[j++] = left[i];

        }

        for (int i = 0; i < right.length; i++) {
            moves[i + middleIndex] = right[i];
        }

        return moves;

    }

    public void drawWaves() {
        Graphics2D g = _robot.getGraphics();

        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;


        //System.out.println("LOW BEARING: " + lowBearing + " HIGH BEARING: " + highBearing + " ABSBEARING: " + Utils.normalAbsoluteAngle(absBearing));

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        double enemyHeading = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians; // smallOffset + (_radarScanner._lastScan == null ? 0 : _radarScanner.nme.bearingRadians);

        double graphWidth = 300;
        int graphx = 30, graphy = 100, cnt = 0; //(int)_robot.getBattleFieldWidth()-((int)graphWidth+30), graphy=100, cnt=0;

/*

        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy, graphx + ((int) graphWidth), graphy);
        g.drawLine(graphx, graphy, graphx, graphy + 100);
        g.drawLine(graphx + ((int) graphWidth), graphy, graphx + (int) graphWidth, graphy + 100);

        if (waves.size() > 0) {

            double bestdist = Double.MAX_VALUE;
            int minIndex = 0;
            int cTime = (int) _robot.getTime();

            for (int i = 0; i < waves.size(); i++) {
                NNBullet w = waves.get(i);
                double dist = CTUtils.bulletVelocity(w.power) * (cTime - w.fireTime);

                if (dist > bestdist && dist < 1000) {
                    minIndex = i;
                    bestdist = dist;
                }

            }
            NNBullet closest = waves.get(minIndex);


            RobotState[] moves = closest.moves;
            Point2D.Double fireLocation = new Point2D.Double(closest.startX, closest.startY);

            double lowBearing = CTUtils.absoluteBearing(fireLocation, moves[0].location);
            double highBearing = CTUtils.absoluteBearing(fireLocation, moves[moves.length - 1].location);

            double lowOffsetAngle = lowBearing - absBearing;
            double highOffsetAngle = highBearing - absBearing;

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            Point2D.Double low = CTUtils.project(fireLocation, lowBearing, _radarScanner.nme.distance);
            g.drawLine((int) closest.startX, (int) closest.startY, (int) low.getX(), (int) low.getY());

            Point2D.Double high = CTUtils.project(fireLocation, highBearing, _radarScanner.nme.distance);
            g.drawLine((int) closest.startX, (int) closest.startY, (int) high.getX(), (int) high.getY());


            int bestDensityIndex = 0;
            int lowestDensityIndex = 0;
            int lowestGFIndex = GF_ZERO;
            double bestDensity = -Double.MAX_VALUE, lowestDensity = Double.MAX_VALUE;

            for (int i = closest.outputs.length; --i >= 0; ) {

                double density = 0;
                double u;
                for (int j = closest.outputs.length; --j >= 0; ) {
                    //double kdeDiff = points[i].danger - points[j].danger;
                    double kdeDiff = closest.outputs[j] - closest.outputs[i];
                    //double kdeDiff = Math.sqrt(Math.pow(points[i].danger - points[j].danger,2));
                    //double kdeDiff = Math.abs(bestNodes.get(i).Distance - bestNodes.get(j).Distance);

                    density += Math.exp(
                            (
                                    u = (kdeDiff) /// 2 // BAND_WIDTH OF 4 seems ok
                            )
                                    * u
                                    * -0.5d
                    );
                }

                if (closest.outputs[i] < closest.outputs[lowestGFIndex])
                    lowestGFIndex = i;

                if (density > bestDensity) {
                    bestDensityIndex = i;
                    bestDensity = density;
                }

                if (density < lowestDensity) {
                    lowestDensityIndex = i;
                    lowestDensity = density;
                }

            }


            int plotpt = graphx + (int) (((double) closest.lowGF + 1) / 2.0 * graphWidth);

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            g.drawLine(plotpt, graphy, plotpt, graphy + 100);

            plotpt = graphx + (int) (((double) closest.highGF + 1) / 2.0 * graphWidth);

            g.setColor(new Color(0.47843137f, 0.33333334f, 0.16078432f, 0.8f));
            g.drawLine(plotpt, graphy, plotpt, graphy + 100);


            int bestGF = (int) Math.round((closest.getGuessFactor(ex, ey) + 1) * GFGun.GF_ZERO);
            int bestGFPlotPoint1 = graphx + (int) (bestGF * (graphWidth / GF_ONE));


            g.setColor(new Color(1.0f, 0.9137255f, 0.14509805f, 0.8f));
            g.drawLine(bestGFPlotPoint1, graphy, bestGFPlotPoint1, graphy + 100);

            double guessfactor = (double) (lowestGFIndex - GF_ZERO) / (double) GF_ZERO;
            double angleOffset = direction * newWave.maxEscapeAngle / guessfactor;

            int bestGFPlotPoint2 = graphx + (int) (((double) closest.fireIndex / GF_ONE) * graphWidth);

            g.setColor(new Color(246, 15, 136, (int) (0.8 * 255)));
            g.drawLine(bestGFPlotPoint2, graphy, bestGFPlotPoint2, graphy + 100);

            g.setColor(new Color(196, 175, 246, (int) (0.8 * 255)));
            g.setFont(new Font("Verdana", Font.PLAIN, 10));
            g.drawString("Chosen GF", bestGFPlotPoint2 - 20, graphy - 10);
            g.drawString("Correct GF", bestGFPlotPoint1 - 25, graphy - 20);
            g.drawString("Closest Wave GF Plot", graphx, graphy + 105);

            for (int i = 0; i < closest.outputs.length; i++) {
                g.setColor(Color.MAGENTA);
                g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 1, graphy + (int) (closest.outputs[i] * 10), 2, 2);

                g.setColor(Color.GREEN);
                if (i == bestDensityIndex)
                    g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 2, graphy + (int) (closest.outputs[i] * 10), 4, 4);

                g.setColor(Color.CYAN);
                if (i == lowestDensityIndex)
                    g.drawOval(graphx + (int) (cnt * (graphWidth / GF_ONE)) - 2, graphy + (int) (closest.outputs[i] * 10), 4, 4);

                cnt++;
            }

        }
        */

        for (int i = 0; i < waves.size(); i++) {
            NNBullet w = (NNBullet) (waves.get(i));
            Point2D.Double center = new Point2D.Double(w.startX, w.startY);

            //int radius = (int)(w.distanceTraveled + w.bulletVelocity);

            double angleDivision = (w.maxEscapeAngle * 2 / w.outputs.length);
            int radius = (int) (w.getBulletSpeed() * (_robot.getTime() - w.fireTime));


            //Point2D.Double center = w.fireLocation;
            g.setColor(java.awt.Color.red);
            if (radius - 40 < center.distance(_radarScanner.nme.location))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            radius += w.getBulletSpeed();
            g.setColor(new Color(1, 0, 1, 0.5f));
            if (radius - 40 < center.distance(_radarScanner.nme.location))
                g.drawOval((int) (center.x - radius), (int) (center.y - radius), radius * 2, radius * 2);

            g.setColor(java.awt.Color.green);
            int cTime = (int) _robot.getTime();


            for (int p = 0; p < w.outputs.length; p++) {
                double angleOffset = angleDivision * (double) p * 0;

                float shade = (float) w.outputs[p];
                shade = (float) CTUtils.clamp(shade * 10, 0.0, 1.0);
                g.setColor(new Color(0, shade, 1, 1.0f));

                //System.out.print(shade + " ");
                //System.out.println("DA: " + w.directAngle + ", AD: " + angleDivision + ", MEA: " + w.maxEscapeAngle);
                Point2D.Double p2 = CTUtils.project(center, (w.directAngle + Math.PI) + (angleDivision * p) - (angleDivision * (w.outputs.length / 2)), (int) ((cTime - w.fireTime) * w.getBulletSpeed()));
                Point2D.Double p3 = CTUtils.project(p2, (w.directAngle + Math.PI) + (angleDivision * p) - (angleDivision * (w.outputs.length / 2)), (int) (w.getBulletSpeed()));

//                g.drawOval((int) (p2.x - 1), (int) (p2.y - 1), 2, 2);
                g.drawLine((int) p2.getX(), (int) p2.getY(), (int) p3.getX(), (int) p3.getY());
            }
        }


    }

    public void update() {

        drawWaves();

        double absBearing = _robot.getHeadingRadians() + _radarScanner.nme.bearingRadians;

        // find our enemy's location:
        double ex = _robot.getX() + Math.sin(absBearing) * _radarScanner.nme.distance;
        double ey = _robot.getY() + Math.cos(absBearing) * _radarScanner.nme.distance;

        // Let's process the waves now:
        for (int i = 0; i < waves.size(); i++) {
            NNBullet currentWave = (NNBullet) waves.get(i);

            if (currentWave.checkHit(ex, ey, _robot.getTime())) {


                if (currentWave.inputs.length != INPUT_LENGTH || currentWave.outputs.length != OUTPUT_LENGTH) {
                    System.out.println("Inputs length mismatch - should be " + currentWave.inputs.length);
                    continue;
                }

                if (currentWave.actualHit) {
                    _hitQueueInputs.add(currentWave.inputs.clone());
                    _hitQueueOutputs.add(currentWave.outputs.clone());

                    if (_hitQueueInputs.size() > 15) {
                        _hitQueueOutputs.remove(0);
                        _hitQueueInputs.remove(0);
                    }
                }
                // Train based on waves.outputs
                // waves.inputs
                // waves.outputs (2 double arrays for training)
                System.out.println("INP=" + Arrays.toString(currentWave.inputs));
                System.out.println(Arrays.toString(currentWave.outputs));

                _theData.clear();
                _theData.add(new BasicMLDataPair(new BasicMLData(currentWave.inputs), new BasicMLData(currentWave.outputs)));

                for (int k = 0; k < _hitQueueInputs.size(); k++) {
                    //data.add(new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k))));
                    assert _hitQueueInputs.get(k).length == INPUT_LENGTH : "INPUT LENGTH MISMATCH!";
                    assert _hitQueueOutputs.get(k).length == OUTPUT_LENGTH : "OUTPUT LENGTH MISMATCH!";

                    //input[k+1] = _hitQueueInputs.get(k);
                    //output[k+1] = _hitQueueOutputs.get(k);
                    _theData.add(new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k))));

                }

                _randomDataBuffer.add(new BasicMLDataPair(new BasicMLData(currentWave.inputs), new BasicMLData(currentWave.outputs)));

                if (_randomDataBuffer.size() > 200)
                {
                    int randIndex = (int)(_rand.nextDouble() * 199);

                    _randomDataBuffer.remove(randIndex);
                }

                _randomData.clear();

                if (_randomDataBuffer.size() > 5) {
                    for (int j = 0; j < 5; j++) {
                        int randIndex = (int)(_rand.nextDouble() * 4.9);

                        _randomData.add(_randomDataBuffer.get(randIndex));
                    }
                }

                if (currentWave.actualHit)
                {
                    int k = _hitQueueInputs.size()-1;
                    _randomData.add(new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k))));

                }



                //MLDataSet mld = new BasicMLDataSet(input, output);

//System.out.println("Data set size is " + mld.size());
                //basicTrain.pause();
                //basicTrain.setTraining(mld);
                basicTrain.iteration(10);
                randomTrain.iteration(10);

                System.out.println("Basic Error: " + Format.formatPercent(basicTrain.getError()));
                System.out.println("Random Error: " + Format.formatPercent(randomTrain.getError()));

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

    public double[] getInputForSituation(Situation s) {

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
        double[] fdistance = RBFUtils.processDataIntoFeatures(Math.min(s.Distance, 800), 200, RBFUtils.getCenters(0, 800, 11));

        // Lateral Velocity - Range 0 - 8, split into 8 features
        double[] flatvel = RBFUtils.processDataIntoFeatures(s.LateralVelocity * 8.0, 8.0, RBFUtils.getCenters(0, 8, 8));

        // Acceleration - Range 0 - 1.0, split into 11 features
        double[] faccel = RBFUtils.processDataIntoFeatures(s.Acceleration, 1.0, RBFUtils.getCenters(0, 1.0, 11));

        // Advancing Velocity - Range -8.0 - 8.0, split into 9 features
        double advancingVelocity = Math.min(16d, Math.max(-16d, s.AdvancingVelocity * 16d)); // +/- 8.0
        double[] fadvancevel = RBFUtils.processDataIntoFeatures(advancingVelocity, 16.0, RBFUtils.getCenters(-8d, +8d, 9));

        // Need distance delta
        double[] fdistlast10 = RBFUtils.processDataIntoFeatures(s.DistanceLast10, 80, RBFUtils.getCenters(0, 80, 6));


        // SinceVelocityChange - Range 0 - 1, split into 7 features
        double[] fsincevelch = RBFUtils.processDataIntoFeatures(s.SinceVelocityChange, 0.5, RBFUtils.getCenters(0, 1, 7));

        // Wall Tries Forward - Range 0.0 - 20.0, split into 7 features
        double[] ffwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesForward * 20, 20.0, RBFUtils.getCenters(0, 20, 7));

        // Wall Tries Backward - Range 0.0 - 20.0, split into 4 features
        double[] fbwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesBack * 20, 20.0, RBFUtils.getCenters(0, 20, 4));

        // Current guess factor - Range -1.0 - 1.0, split into 11 features
        //double[] fcurgf = RBFUtils.processDataIntoFeatures(currentGuessFactor, 0.5, RBFUtils.getCenters(-1.0, 1.0, 11));

        return RBFUtils.mergeFeatures(fdistance, flatvel, faccel, fadvancevel, fdistlast10, fsincevelch, ffwalltries, fbwalltries); //, fcurgf);

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

    public int getCentroid(double[] results) {
        int bestIndex = 0;
        double bestDensity = 0;

        for (int i = results.length - 1; --i >= 0; ) {

            double density = 0;
            double u;
            for (int j = results.length; --j >= 0; ) {
                double kdeDiff = (results[i] - results[j])*1000;
                //double kdeDiff = Math.abs(bestNodes.get(i).Distance - bestNodes.get(j).Distance);

                density += Math.exp((u = (kdeDiff)  /* BAND_WIDTH */) * u * -0.5d);
            }


            if (density > bestDensity) {
                bestDensity = density;
                bestIndex = i;
            }
        }

        return bestIndex;

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
        double[] situationInput = getInputForSituation(s);

        assert situationInput.length == INPUT_LENGTH : "Situation input length doesn't match";

        newWave = new NNBullet(x, y, absBearing, ((Double) _radarScanner._surfAbsBearings.get(0)).doubleValue(), _radarScanner.FIRE_POWER,
                direction, _robot.getTime(), situationInput);


        int bestGF = GF_ZERO;    // initialize it to be in the middle, guessfactor 0.

        // TODO: USE NEURAL NET TO FIND BETTER GF
        //System.out.println("Basic: " + Format.formatPercent(basicTrain.getError()));

        BasicMLData inp = new BasicMLData(situationInput);

        double[] outgf = basicNetwork.compute(inp).getData();
        double[] randoutgf = randomNetwork.compute(inp).getData();
        //System.out.println("Guessfactor Output: " + Arrays.toString(outgf));


        int maxgf = 31;
        for (int i = 0; i < outgf.length; i++) {
            //outgf[i] = 1.0 - outgf[i];

            if (outgf[i] + randoutgf[i] > outgf[maxgf] + randoutgf[maxgf]) {
                maxgf = i;
            }
        }


        bestGF = maxgf;
        newWave.outputs = outgf; // Temporarily use this as outputs
        //bestGF = getCentroid(outgf);

        newWave.fireIndex = bestGF;
        newWave.moves = getEnemyMoves();

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double) (bestGF - GF_ZERO) / (double) GF_ZERO;
        currentGuessFactor = guessfactor;

        double angleOffset = direction < 0 ? -(guessfactor * newWave.lowGF * newWave.maxEscapeAngle) :
                (guessfactor * newWave.highGF * newWave.maxEscapeAngle);

        return angleOffset;


    }
}
