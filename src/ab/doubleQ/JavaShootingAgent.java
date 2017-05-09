package ab.doubleQ;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.utils.StateUtil;
import ab.vision.ABType;
import ab.vision.GameStateExtractor;
import ab.vision.Vision;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.Serializable;
import java.util.*;
import java.util.List;

/**
 * Created by jeehoon on 27/04/2017.
 */
@SuppressWarnings("ALL")
public class JavaShootingAgent {
    private JavaTrajectoryPlanner trajectoryPlanner;
    private ActionRobot actionRobot;
    private Rectangle sling;

    private ABType prevBirdType;
    private State prevState;
    private int prevScore;

    // a standalone implementation of the Naive Agent
    public JavaShootingAgent() {
        actionRobot = new ActionRobot();
        trajectoryPlanner = new JavaTrajectoryPlanner();
        prevScore = 0;
        prevState = new State();
        // --- go to the Poached Eggs episode level selection page ---
        ActionRobot.GoFromMainMenuToLevelSelection();
    }

    public class State implements Serializable {
        private List<Action> actionSequence;

        public State(Action... actions) {
            actionSequence = new ArrayList<Action>();
            for (Action action : actions) {
                actionSequence.add(action);
            }
        }
        public List<Action> getActionSequence() {
            return actionSequence;
        }
    }

    public class Action implements Serializable {
        private int angle;
        private int power;
        private int tabInterval;

        public Action(int angle, int power, int tabInterval) {
            this.angle = angle;
            this.power = power;
            this.tabInterval = tabInterval;
        }
        public int getAngle() {
            return angle;
        }
        public int getPower() {
            return power;
        }
        public int getTabInterval() {
            return tabInterval;
        }
    }

    public class Observation implements Serializable {
        private State state;
        private int reward;
        private Action action;
        private boolean terminal;

        public Observation(State state, int reward, Action action, boolean terminal) {
            this.state = state;
            this.reward = reward;
            this.action = action;
            this.terminal = terminal;
        }
        public State getState() {
            return state;
        }
        public int getReward() {
            return reward;
        }
        public Action getAction() {
            return action;
        }
        public boolean isTerminal() {
            return terminal;
        }
    }

    public void loadLevel(int targetLevel) {
        actionRobot.loadLevel(targetLevel);
    }

    public GameStateExtractor.GameState getState() {
        return actionRobot.getState();
    }

    public void goFromMainMenuToLevelSelection() {
        ActionRobot.GoFromMainMenuToLevelSelection();
    }

    public void setReady() {
        assert (actionRobot.getState() == GameStateExtractor.GameState.PLAYING);
        BufferedImage screenshot = ActionRobot.doScreenShot();
        Vision vision = new Vision(screenshot);
        sling = vision.findSlingshotMBR();
    }

    public boolean isReady() {
        return sling != null;
    }

    public boolean isSlingChanged() {
        ActionRobot.fullyZoomOut();
        BufferedImage screenshot = ActionRobot.doScreenShot();
        Vision vision = new Vision(screenshot);
        Rectangle _sling = vision.findSlingshotMBR();

        return  _sling == null ||
                _sling.width  != sling.width ||
                _sling.height != sling.height;
    }

    public Observation shoot(int angle, int power, int tabInterval) {
        cshoot(angle, power, tabInterval);

        int curScore = StateUtil.getScore(ActionRobot.proxy);
        int reward = curScore - prevScore;
        prevScore = curScore;

        Action curAction = new Action(angle, power, tabInterval);
        Observation observation;

        if (reward > 0) {
            Action[] actions = new Action[prevState.getActionSequence().size()];
            State curState = new State(prevState.getActionSequence().toArray(actions));
            curState.getActionSequence().add(curAction);

            observation = new Observation(prevState, reward, curAction, false);
            prevState = curState;

        } else {
            observation = new Observation(prevState, reward, curAction, false);
        }
        return observation;
    }

    private void cshoot(int angle, int power, int tabInterval) {
        while (!isReady() || isSlingChanged()) {
            setReady();
        }
        Point releasePoint = trajectoryPlanner.findReleasePoint(
                sling,
                angle / (double) 100 * Math.PI / (double) 2,
                power / (double) 100);
        Point refPoint = trajectoryPlanner.getReferencePoint(sling);
        int dx = (int) releasePoint.getX() - refPoint.x;
        int dy = (int) releasePoint.getY() - refPoint.y;
        Shot shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tabInterval);
        actionRobot.cshoot(shot);
    }

    private int getNumPigs() {
        return new Vision(ActionRobot.doScreenShot()).findPigsMBR().size();
    }
}
