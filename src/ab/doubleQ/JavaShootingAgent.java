package ab.doubleQ;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.utils.StateUtil;
import ab.vision.ABType;
import ab.vision.GameStateExtractor;
import ab.vision.Vision;
import ab.vision.GameStateExtractor.GameState; // added by haeyong.k

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.Serializable;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.*;
import java.util.List;

import py4j.*;

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
    private int initNumBirds;
    private int initNumBlocks;
    private int initScreenSize;

    //public GameState game_state;

    // a standalone implementation of the Naive Agent
    public JavaShootingAgent() {
        actionRobot = new ActionRobot();
        trajectoryPlanner = new JavaTrajectoryPlanner();
        prevScore = 0;
        prevState = new State();
        // << go to the Poached Eggs episode level selection page >>
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
        private byte[] screen;
        private int reward;
        private Action action;
        private boolean terminal;

        //public Observation(State state, byte[] screen, int reward, Action action, boolean terminal) {
	public Observation(byte[] screen, int reward, Action action, boolean terminal) {
            this.state = state;
            this.screen = screen;
            this.reward = reward;
            this.action = action;
            this.terminal = terminal;
        }
        public State getState() {
            return state;
        }
        public byte[] getScreen() {
            return screen;
        }
        public int getReward() {
            return reward;
        }
        public Action getAction() {
            return action;
        }
        public boolean getTerminal() {
            return terminal;
        }
    }

    public Observation loadLevel(int targetLevel) {
        actionRobot.loadLevel(targetLevel);
        initNumBirds = getNumBirds();
        prevScore = 0;
        prevState = new State();

        return new Observation(getScreen(), 0, null, false);
    }

    public GameStateExtractor.GameState getGameState() {
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

    public int getScreenWidth() {
        return ActionRobot.doScreenShot().getWidth();
    }
    public int getScreenHeight() {
        return ActionRobot.doScreenShot().getHeight();
    }

    public byte[] getScreen() {
        int[][] scene = new Vision(ActionRobot.doScreenShot()).getMBRVision()._scene;
        return convert2DArrayToBytes(scene);
    }

    public int getMaxStep() {
        return initNumBirds;
    }

    public Observation shoot(int angle, int power, int tabInterval) {
        cshoot(angle, power, tabInterval);

        int curScore = StateUtil.getScore(ActionRobot.proxy);
        int reward = curScore - prevScore;
	//System.out.println(String.format("curScore=%d",curScore));
	//System.out.println(String.format("prevScore=%d", prevScore));
	//System.out.println(String.format("reward=%d", reward));
	
        prevScore = curScore;

        int numPigs = getNumPigs();
        int numBirds = getNumBirds();
        boolean isTerminal = (numPigs == 0) || (numBirds == 0) ;
	//boolean isTerminal = (numPigs == 0) || (numBirds == 0) || (actionRobot.getState() == GameStateExtractor.GameState.WON);

	//State state = actionRobot.getState();
        Action curAction = new Action(angle, power, tabInterval);
        //Observation observation = new Observation(getScreen(), reward, curAction, isTerminal);
	
	//State state = new State(prevState.getActionSequence().toArray(actions));
	//Observation observation = new Observation(state,getScreen(), reward, curAction, isTerminal);
	Observation observation = new Observation(getScreen(), reward, curAction, isTerminal);

	boolean isStateGame = (actionRobot.getState() == GameState.WON);
        System.out.println(String.format("isTerminal=%s",isTerminal));
	
	//System.out.println(String.format("isStateGame=%s", isStateGame));
	System.out.println(actionRobot.getState());
	
	curScore = StateUtil.getScore(ActionRobot.proxy);
	System.out.println(String.format("curScore=%d",curScore));
		
        if (reward > 0) {
            Action[] actions = new Action[prevState.getActionSequence().size()];
            State curState = new State(prevState.getActionSequence().toArray(actions));
            curState.getActionSequence().add(curAction);
            prevState = curState;
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
    public int getNumBirds() {
        return new Vision(ActionRobot.doScreenShot()).findBirdsMBR().size();
    }

    private byte[] convert2DArrayToBytes(int[][] scene) {
        ByteBuffer byteBuffer = ByteBuffer.allocate(scene.length * scene[0].length * 4);
        IntBuffer intBuffer = byteBuffer.asIntBuffer();
        for (int i=0; i<scene.length; i++) {
            intBuffer.put(scene[i]);
        }

        return byteBuffer.array();
    }
}
