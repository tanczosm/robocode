package enemy;

import robocode.AdvancedRobot;
import robocode.HitRobotEvent;

import java.awt.*;

public class CirclingBot extends AdvancedRobot {

    public void run() {
        setAllColors(Color.green);

        while (true) {
            setTurnRight(Double.POSITIVE_INFINITY);
            setMaxVelocity(5);
            ahead(Double.POSITIVE_INFINITY);
        }
    }

    public void onHitRobot(HitRobotEvent e) {
        if (e.isMyFault()) {
            turnRight(10);
        }
    }
}
