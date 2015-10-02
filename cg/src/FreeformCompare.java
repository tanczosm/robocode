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

import java.util.Arrays;

public class FreeformCompare {

    public static final boolean dualHidden = true;
    public static final int ITERATIONS = 175;

    public static BasicNetwork basicNetwork;
    public static FreeformNetwork freeformNetwork;

    /**
     * The input necessary for XOR.
     */
    public static double XOR_INPUT[][] = {{0.0, 0.0}, {1.0, 0.0},
            {0.0, 1.0}, {1.0, 1.0}};

    /**
     * The ideal data necessary for XOR.
     */
    public static double XOR_IDEAL[][] = {{0.0}, {1.0}, {1.0}, {0.0}};

    public static void main(String[] args) {

        // create the basic network
        basicNetwork = new BasicNetwork();
        basicNetwork.addLayer(new BasicLayer(null, true, 2));
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 2));
        if (dualHidden) {
            basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), true, 3));
        }
        basicNetwork.addLayer(new BasicLayer(new ActivationSigmoid(), false, 1));
        basicNetwork.getStructure().finalizeStructure();
        basicNetwork.reset();
        basicNetwork.reset(1000);

        // create the freeform network
        freeformNetwork = new FreeformNetwork(basicNetwork);

        // create training data
        MLDataSet trainingSet = new BasicMLDataSet(new double[1][2], new double[1][1]);

        // create two trainers
        FreeformBackPropagation freeformTrain = new FreeformBackPropagation(freeformNetwork, trainingSet, 0.7, 0.3);
        Backpropagation basicTrain = new Backpropagation(basicNetwork, trainingSet, 0.7, 0.3);

        freeformTrain.setBatchSize(1);
        basicTrain.setBatchSize(1);

        int sample = 0;

        // perform both
        for (int i = 1; i <= ITERATIONS; i++) {
            //if (sample < XOR_INPUT.length)
            trainingSet.add(new BasicMLData(XOR_INPUT[sample % XOR_INPUT.length]), new BasicMLData(XOR_IDEAL[sample % XOR_IDEAL.length]));

            freeformTrain.iteration();
            basicTrain.iteration();
            System.out.println("Iteration #" + i + " : "
                    + "Freeform: " + Format.formatPercent(freeformTrain.getError())
                    + ", Basic: " + Format.formatPercent(basicTrain.getError()));

            BasicMLData inp = new BasicMLData(XOR_INPUT[2]);

            System.out.println("Record count: " + trainingSet.getRecordCount());

            System.out.println(Arrays.toString(basicNetwork.compute(inp).getData()));
            System.out.println(Arrays.toString(freeformNetwork.compute(inp).getData()));

            sample++;
            System.out.println(basicNetwork.compute(inp));
        }
    }
}