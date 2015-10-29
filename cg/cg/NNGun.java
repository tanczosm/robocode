package cg;

import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.mathutil.error.ErrorCalculation;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.basic.BasicMLDataPair;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.neural.networks.training.propagation.back.Backpropagation;
import org.encog.util.Format;
import org.encog.util.obj.SerializeObject;
import robocode.AdvancedRobot;
import robocode.RobocodeFileOutputStream;

import java.awt.*;
import java.awt.geom.Point2D;
import java.io.*;
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

    public static final int INPUT_LENGTH = 70;
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
    private double _errAvg;

    public PrintStream fileWriter = null;

    double lastInput;
    boolean isDuplicate = false;

    boolean saveNetwork = false;

    public NNGun(AdvancedRobot robot, RadarScanner radarScanner) {
        _robot = robot;
        _radarScanner = radarScanner;
        _hitQueueInputs = new ArrayList<double[]>();
        _hitQueueOutputs = new ArrayList<double[]>();
        _theData = new ArrayList<MLDataPair>();
        _randomData = new ArrayList<MLDataPair>();
        _randomDataBuffer = new ArrayList<MLDataPair>();

        _rand = new Random();
        if (saveNetwork)
            basicNetwork = loadNetwork("basicNetwork.dat");

        if (basicNetwork != null)
            System.out.println("Network #1 online and ready to rock");

        if (basicNetwork == null) {
            // create the basic network
            basicNetwork = new BasicNetwork();
            basicNetwork.addLayer(new BasicLayer(null, true, INPUT_LENGTH));
            //basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 39));
            basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, OUTPUT_LENGTH));
            basicNetwork.getStructure().finalizeStructure();
            basicNetwork.reset();
            basicNetwork.reset(1000);
        }
        if (saveNetwork)
            randomNetwork = loadNetwork("randomNetwork.dat");

        if (randomNetwork != null)
            System.out.println("Network #2 online and ready to rock");

        if (randomNetwork == null) {
            // create the basic network
            randomNetwork = new BasicNetwork();
            randomNetwork.addLayer(new BasicLayer(null, true, INPUT_LENGTH));
            //basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 39));
            randomNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, OUTPUT_LENGTH));
            randomNetwork.getStructure().finalizeStructure();
            randomNetwork.reset();
            randomNetwork.reset(1000);
        }

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

        setRobot();
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

    public void drawFactor(double[] data, int factorStart, int featureCount, String featureName, int topx, int topy, int position) {
        Graphics2D g = _robot.getGraphics();

        double graphWidth = 150;
        int height = 45;

        topy = (height + 5) * position;
        int graphx = topx, graphy = topy + height - 10, cnt = 0;
        int rightBorder = graphx + ((int) graphWidth - (int) (graphWidth / featureCount));
        g.setColor(Color.GREEN);
        g.drawLine(graphx, graphy, rightBorder, graphy);
        g.drawLine(graphx, graphy, graphx, graphy + (height - 10));
        g.drawLine(rightBorder, graphy, rightBorder, graphy + (height - 10));

        g.setColor(Color.white);
        Point2D.Double lastpoint = new Point2D.Double(graphx, graphy);

        g.setFont(new Font("Verdana", Font.PLAIN, 10));
        g.drawString(featureName, graphx, graphy - 10);

        for (int i = factorStart; i < factorStart + featureCount && i < data.length; i++) {
            g.setColor(Color.MAGENTA);
            //g.drawOval(graphx + (int) (cnt * (graphWidth / featureCount)) - 1, graphy + (int) (data[i] * 20), 2, 2);
            Point2D.Double nextpoint = new Point2D.Double(graphx + (int) (cnt * (graphWidth / featureCount)) - 1, graphy + (int) (data[i] * (height - 10)));
            g.drawLine((int) lastpoint.x, (int) lastpoint.y, (int) nextpoint.x, (int) nextpoint.y);

            lastpoint = nextpoint;

            cnt++;
        }

        //Point2D.Double nextpoint = new Point2D.Double(graphx + (int) graphWidth, graphy);
        //g.drawLine((int)lastpoint.x, (int)lastpoint.y, (int)nextpoint.x, (int)nextpoint.y);

    }

    public void drawWaves() {

        if (waves.size() > 0) {

            double[] inputs = null;


            for (int p = 0; p < waves.size(); p++)

                if (waves.get(p).isReal) {
                    inputs = waves.get(p).inputs;

                    /*
                    if (inputs[1] == lastInput)
                        isDuplicate = true;

                    if (!isDuplicate)
                        writeLogLine(inputs);
                    */
                    //inputs[1] = lastInput;

                    isDuplicate = false;
                    break;
                }

            if (inputs != null) {
                drawFactor(inputs, 0, 11, "Distance", 0, 0, 0);
                drawFactor(inputs, 11, 8, "Lateral Velocity", 0, 0, 1);
                drawFactor(inputs, 11 + 8, 11, "Acceleration", 0, 0, 2);
                drawFactor(inputs, 11 + 8 + 11, 9, "Advancing Velocity", 0, 0, 3);
                drawFactor(inputs, 11 + 8 + 11 + 9, 6, "Dist Last 10 Ticks", 0, 0, 4);
                drawFactor(inputs, 11 + 8 + 11 + 9 + 6, 7, "Time Since Vel Change", 0, 0, 5);
                drawFactor(inputs, 11 + 8 + 11 + 9 + 6 + 7, 7, "Since Direction Change", 0, 0, 6);
                drawFactor(inputs, 11+8+11+9+6+7+7, 7, "Forward Wall Radians", 0, 0, 7);
                drawFactor(inputs, 11+8+11+9+6+7+7+7, 4, "Reverse Wall Radians", 0, 0, 8);

            }

        }

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

        for (int i = 0; i < waves.size(); i++) {

            NNBullet w = (NNBullet) (waves.get(i));

            if (!w.isReal)
                continue;

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

*/
    }

    /**
     * Calculate the error for this neural network. The error is
     calculated using root-mean-square(RMS).
     * @param pair The training set data pair
     * @return The error percentage.
     */
    public double calculateError(final BasicMLDataPair pair){
        final ErrorCalculation errorCalculation=new ErrorCalculation();
        final double[] actual=new double[OUTPUT_LENGTH];

        basicNetwork.compute(pair.getInputArray(),actual);
        errorCalculation.updateError(actual,pair.getIdealArray(),pair.getSignificance());

        return errorCalculation.calculate();
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
                    //System.out.println("Inputs length mismatch - should be " + currentWave.inputs.length);
                    continue;
                }
                /*
                if (currentWave.actualHit && currentWave.isReal)
                {
                    for (int k = 0; k < currentWave.outputs.length; k++)
                    {
                        currentWave.outputs[k] = 1.0 - currentWave.outputs[k];
                    }
                }
                */
                if (currentWave.isReal) {
                    _hitQueueInputs.add(currentWave.inputs.clone());
                    _hitQueueOutputs.add(currentWave.outputs.clone());

                    if (_hitQueueInputs.size() > 5) {
                        _hitQueueOutputs.remove(0);
                        _hitQueueInputs.remove(0);
                    }
                }
                // Train based on waves.outputs
                // waves.inputs
                // waves.outputs (2 double arrays for training)
                //System.out.println("INP=" + Arrays.toString(currentWave.inputs));
                //System.out.println(Arrays.toString(currentWave.outputs));

                // First let's train the wave that just came in
                _theData.clear();
                _theData.add(new BasicMLDataPair(new BasicMLData(currentWave.inputs), new BasicMLData(currentWave.outputs)));

                if (_theData.size() > 0)
                    basicTrain.iteration(1);

                // Let's train the random network

                _randomDataBuffer.add(new BasicMLDataPair(new BasicMLData(currentWave.inputs), new BasicMLData(currentWave.outputs)));

                if (_randomDataBuffer.size() > 200) {
                    int randIndex = (int) (_rand.nextDouble() * 199);

                    _randomDataBuffer.remove(randIndex);
                }

                _randomData.clear();

                int fillCount = 5;

                if (currentWave.isReal) {
                    _randomData.add(new BasicMLDataPair(new BasicMLData(currentWave.inputs), new BasicMLData(currentWave.outputs)));
                    fillCount--;
                }

                if (_randomDataBuffer.size() > 5) {
                    for (int j = 0; j < fillCount; j++) {
                        int randIndex = (int) (_rand.nextDouble() * _randomDataBuffer.size());

                        _randomData.add(_randomDataBuffer.get(randIndex));
                    }
                }

                if (_randomData.size() > 0)
                    randomTrain.iteration(1);

                // Done training the random network

                /*
                if (currentWave.actualHit) {
                    int k = _hitQueueInputs.size() - 1;
                    _randomData.add(new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k))));

                }*/

                _theData.clear();

                double errSum = 0;
                int count = 0, errCount = 0;

                do {

                    for (int k = 0; k < _hitQueueInputs.size(); k++) {
                        //data.add(new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k))));
                        assert _hitQueueInputs.get(k).length == INPUT_LENGTH : "INPUT LENGTH MISMATCH!";
                        assert _hitQueueOutputs.get(k).length == OUTPUT_LENGTH : "OUTPUT LENGTH MISMATCH!";

                        BasicMLDataPair pair = new BasicMLDataPair(new BasicMLData(_hitQueueInputs.get(k)), new BasicMLData(_hitQueueOutputs.get(k)));

                        double err = calculateError(pair);
                        //System.out.println("Err: " + err);

                        if (err > 0.05) {
                            errSum += err;
                            errCount++;
                            _theData.add(pair);
                        }

                    }

                    if (_theData.size() > 0)
                        basicTrain.iteration(1);

                    _errAvg = (errCount == 0 ? 0 : errSum / errCount);
                    count++;
                }
                while (count < 2 && _errAvg > 0.05);

                //System.out.println("Average Error: " + _errAvg);





                //System.out.println("Basic Error: " + Format.formatPercent(basicTrain.getError()));
                //System.out.println("Random Error: " + Format.formatPercent(randomTrain.getError()));

                waves.remove(currentWave);
                i--;
            }

        }
    }

    public void takeVirtualShot(Situation s, double bearing) {
        super.takeVirtualShot(s, bearing);

        //newWave.startBearing = bearing; // Update bearing to actual bearing..

        newWave.fireTime = _robot.getTime();
        newWave.isReal = true;

       /*if (newWave != null)
            waves.add(newWave);*/

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
        double bft = s.Distance / _radarScanner.FIRE_SPEED;
        //double[] fdistance = RBFUtils.processDataIntoFeatures(Math.min(s.Distance, 800), 800, RBFUtils.getCenters(0, 800, 11));
        double[] fdistance = RBFUtils.processDataIntoFeatures(Math.min(bft, 105), 105, RBFUtils.getCenters(0, 105, 11));

        // Lateral Velocity - Range 0 - 8, split into 8 features
        double[] flatvel = RBFUtils.processDataIntoFeatures(s.LateralVelocity * 8.0, 2.0, RBFUtils.getCenters(0, 8, 8));

        // Acceleration - Range 0 - 1.0, split into 11 features
        double[] faccel = RBFUtils.processDataIntoFeatures(s.Acceleration, 0.05, RBFUtils.getCenters(0, 1.0, 11));

        // Advancing Velocity - Range -8.0 - 8.0, split into 9 features
        double advancingVelocity = Math.min(1.0d, Math.max(-1.0d, s.AdvancingVelocity * 1.0d)); // +/- 8.0
        double[] fadvancevel = RBFUtils.processDataIntoFeatures(advancingVelocity, 0.1, RBFUtils.getCenters(-1.0d, +1.0d, 9));

        // Need distance delta
        double[] fdistlast10 = RBFUtils.processDataIntoFeatures(s.DistanceLast10, 80, RBFUtils.getCenters(0, 80, 6));


        // SinceVelocityChange - Range 0 - 1, split into 7 features
        double[] fsincevelch = RBFUtils.processDataIntoFeatures(s.SinceVelocityChange, 0.05, RBFUtils.getCenters(0, 1, 7));

        /*
        // Wall Tries Forward - Range 0.0 - 20.0, split into 7 features
        double[] ffwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesForward * 20, 20.0, RBFUtils.getCenters(0, 20, 7));

        // Wall Tries Backward - Range 0.0 - 20.0, split into 4 features
        double[] fbwalltries = RBFUtils.processDataIntoFeatures(s.WallTriesBack * 20, 20.0, RBFUtils.getCenters(0, 20, 4));
        */

        // SinceDirectionChange - Range 0 - bft, split into 7 features
        double[] fsincedirch = RBFUtils.processDataIntoFeatures(Math.min(s.SinceDirectionChange, bft), 1, RBFUtils.getCenters(0, bft, 7));

        double wrdf = Math.min(1.5, s.WallRadialDistanceForward);
        double wrdb = Math.min(1.0, s.WallRadialDistanceBack);

        // Forward radians to wall - Range 0.0 - 1.5, split into 7 features
        double[] ffwrdf = RBFUtils.processDataIntoFeatures(wrdf, 0.05, RBFUtils.getCenters(0, 1.5, 7));

        // Back radians to wall - Range 0.0 - 1.0, split into 4 features
        double[] ffwrdb = RBFUtils.processDataIntoFeatures(wrdb, 0.05, RBFUtils.getCenters(0, 1.0, 4));

        //System.out.println("wallfd: " + Math.min(1.5, s.WallRadialDistanceForward) + ", wallbk" + Math.min(1.0, s.WallRadialDistanceBack));

        // Current guess factor - Range -1.0 - 1.0, split into 11 features
        //double[] fcurgf = RBFUtils.processDataIntoFeatures(currentGuessFactor, 1.0, RBFUtils.getCenters(-1.0, 1.0, 11));

        return RBFUtils.mergeFeatures(fdistance, flatvel, faccel, fadvancevel, fdistlast10, fsincevelch, fsincedirch, ffwrdf, ffwrdb);

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
                double kdeDiff = (results[i] - results[j]) * 1000;
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

        // First let's calculate the lowest and highest possible guessfactors

        newWave.moves = getEnemyMoves();

        double lowBearing = CTUtils.absoluteBearing(new Point2D.Double(x, y), newWave.moves[0].location);
        double highBearing = CTUtils.absoluteBearing(new Point2D.Double(x, y), newWave.moves[newWave.moves.length - 1].location);

        int lowGF = (int) Math.floor((newWave.getGuessFactor(newWave.moves[0].location.getX(), newWave.moves[0].location.getY()) + 1) * GF_ZERO);
        int highGF = (int) Math.ceil((newWave.getGuessFactor(newWave.moves[newWave.moves.length - 1].location.getX(), newWave.moves[newWave.moves.length - 1].location.getY()) + 1) * GF_ZERO);

        if (lowGF > highGF) {
            int swap = lowGF;
            lowGF = highGF;
            highGF = swap;
        }
        newWave.lowGF = (double) (lowGF - GF_ZERO) / (double) GF_ZERO;
        newWave.highGF = (double) (highGF - GF_ZERO) / (double) GF_ZERO;

        //System.out.println("LowGF: " + lowGF + ", highGF: " + highGF);


        BasicMLData inp = new BasicMLData(situationInput);

        double[] outgf = basicNetwork.compute(inp).getData();
        double[] randoutgf = randomNetwork.compute(inp).getData();
        //System.out.println("Guessfactor Output: " + Arrays.toString(outgf));

        if (randoutgf.length != outgf.length)
        {
            randoutgf = new double[outgf.length];
        }

        int maxgf = 31;
        for (int i = lowGF; i <= highGF && i < outgf.length; i++) {
            //outgf[i] = 1.0 - outgf[i];

            if (i < 0 || i > outgf.length-1)
                continue;

            if (outgf[i] + randoutgf[i] > outgf[maxgf] + randoutgf[maxgf]) {
                maxgf = i;
            }
        }


        bestGF = maxgf;
        newWave.outputs = outgf; // Temporarily use this as outputs
        //bestGF = getCentroid(outgf);

        newWave.fireIndex = bestGF;


        waves.add(newWave);

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double) (bestGF - GF_ZERO) / (double) GF_ZERO;
        currentGuessFactor = guessfactor;

        /*
        double angleOffset = direction < 0 ? -(guessfactor * newWave.lowGF * newWave.maxEscapeAngle) :
                (guessfactor * newWave.highGF * newWave.maxEscapeAngle);
        */
        double angleOffset = direction < 0 ? -(guessfactor * newWave.maxEscapeAngle) :
                (guessfactor * newWave.maxEscapeAngle);

        return angleOffset;


    }

    public void writeLogLine(double[] inputs) {
        String output = "";

        for (int i = 0; i < inputs.length; i++) {
            output += inputs[i] + ",";
        }

        fileWriter.println(output);
    }

    public void setRobot() {
        /*
        if (fileWriter == null) {
            try {
                fileWriter = new PrintStream(new RobocodeFileOutputStream(_robot.getDataFile("count.csv")));
            } catch (IOException e) {
                //System.out.println("Unable to write to variables log. " + e.getMessage());
            }
        }*/
    }

    public void saveNetwork (String filename, BasicNetwork n)
    {

        try {

            File df = _robot.getDataFile(filename);

            RobocodeFileOutputStream fos = null;
            ObjectOutputStream out = null;

            fos = new RobocodeFileOutputStream(df);
            out = new ObjectOutputStream(fos);
            out.writeObject(n);
            out.close();

        }
        catch (IOException e)
        {
            System.out.println("Unable to serialize object");
        }
    }

    public BasicNetwork loadNetwork (String filename)
    {
        Serializable object = null;
        FileInputStream fis = null;
        ObjectInputStream in = null;

        try {
            fis = new FileInputStream(_robot.getDataFile(filename));
            in = new ObjectInputStream(fis);
            try {
                object = (Serializable) in.readObject();
            }
            catch (ClassNotFoundException cnf)
            {

            }
            in.close();
        }
        catch (IOException e)
        {
            System.out.println("Unable to open \"" + filename + "\"");
        }

        return (BasicNetwork)object;
    }

    public @Override void onBattleEnded() {

        System.out.println("Saving network to disk");
        if (saveNetwork) {
            saveNetwork("basicNetwork.dat", basicNetwork);
            saveNetwork("randomNetwork.dat", randomNetwork);
        }
        //fileWriter.close();
    }
}
