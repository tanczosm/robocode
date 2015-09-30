import org.encog.ml.data.MLDataSet;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.ml.data.basic.BasicMLDataPair;
import org.encog.ml.data.basic.BasicMLData;
import org.encog.ml.data.MLDataPair;
import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import org.encog.util.simple.EncogUtility;

class NNBuilder {
    private BasicNetwork method;

    public NNBuilder() {
        setupNetwork();
    }

    private void setupNetwork() {
        final int GRID_SIDE = 3;
        method = new BasicNetwork();
        method.addLayer(new BasicLayer(null, true, GRID_SIDE * GRID_SIDE));
        method.addLayer(new BasicLayer(new ActivationSigmoid(), true,
                GRID_SIDE));
        method.addLayer(new BasicLayer(new ActivationSigmoid(), false,
                9));
        method.getStructure().finalizeStructure();
        method.reset();
    }

    private BasicMLDataSet generateTrainingSet() {
        LoggingActor act = new LoggingActor(new AlgoAI());
        Player p1 = new Player(act);
        Player p2 = new Player(new RandomAI());

        // play 100k games as the first to place
        for(int i = 0; i < 100000; i++) {
            Game g = new Game(p1, p2);
            while(!g.isGameOver()) {
                g.playTurn();
            }
        }
        // play 100k games as second to place.
        for(int i = 0; i < 100000; i++) {
            Game g = new Game(p2, p1);
            while(!g.isGameOver()) {
                g.playTurn();
            }
        }


        return act.getDataset();
    }


    public void train() {
        System.out.println("trainset gen");
        BasicMLDataSet trainset = generateTrainingSet();
        System.out.println("trainset train");
        EncogUtility.trainToError(method, trainset, 0.05);
    }

    public BasicNetwork getNetwork() {
        return method;
    }
}

public class FeedForwardNNAI extends Actor {

    private BasicNetwork network;

    public FeedForwardNNAI(BasicNetwork network) {
        this.network = network;
    }

    public FeedForwardNNAI() {
        NNBuilder builder = new NNBuilder();

        builder.train();

        this.network = builder.getNetwork();
    }

    public void takeTurn(Game g, Player plr) {
        double[] vars = g.getGrid().generateMLVars(plr);
        BasicMLData indata = new BasicMLData(vars);


        final double[] data = network.compute(indata).getData();
        double[] res = new double[data.length];

        Grid grid = g.getGrid();

        for(int i = 0; i < data.length; i++) {
            if(!grid.getSquare(i).isOccupied()) {
                res[i] = data[i];
            }
        }

        int maxima = 0;
        double val = 0.0;

        for(int i = 0; i < res.length; i++) {
            if(res[i] > val) {
                maxima = i;
                val = res[i];
            }
        }

        g.placePiece(plr, maxima);
    }
}
