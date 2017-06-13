package ab.doubleQ;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.planner.TrajectoryPlanner;
import ab.utils.StateUtil;
import ab.vision.ABObject;
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

    private TrajectoryPlanner tp;
    private Random randomGenerator;
    //public GameState game_state;

    // a standalone implementation of the Naive Agent
    public JavaShootingAgent() {
        actionRobot = new ActionRobot();
        trajectoryPlanner = new JavaTrajectoryPlanner();
        prevScore = 0;
        prevState = new State();

        tp = new TrajectoryPlanner();
        randomGenerator = new Random();
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
        setSlingReady();
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
        return getPigs().size();
    }
    private List<ABObject> getPigs() {
        return new Vision(ActionRobot.doScreenShot()).findPigsMBR();
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

    class ShootInfo {
        double angle;
        double power;
        public ShootInfo(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }
        public double getAngle() {
            //return (int) this.angle * 200 / Math.PI;
	    return this.angle;
        }
        public double getPower() {
            //return (int) this.power * 50;
	    return this.power;
        }
    }

    public ShootInfo getShootInfo() {
        setSlingReady();
        List<ABObject> targetPigs = getPigs();
        ABObject targetPig = targetPigs.get(randomGenerator.nextInt(targetPigs.size()));
        Point targetPoint = targetPig.getCenter();

	// estimate the trajectory
	ArrayList<Point> pts = tp.estimateLaunchPoint(sling, targetPoint);

	Point releasePoint = null;
	// do a high shot when entering a level to find an accurate velocity
	if (pts.size() > 1){
	    releasePoint = pts.get(1);
	}
	else if (pts.size() == 1){
	    releasePoint = pts.get(0);
	}
	else if (pts.size() == 2){
	    // randomly choose between the trajectories, with a 1 in
	    // 6 chance of choosing the high one
	    if (randomGenerator.nextInt(6) == 0)
		releasePoint = pts.get(1);
	    else
		releasePoint = pts.get(0);
	}
	else{
	    if(pts.isEmpty()){
		System.out.println("No release point found for the target");
		System.out.println("Try a shot with 45 degree");
		releasePoint = tp.findReleasePoint(sling, Math.PI/4);
	    }
	}

	 // Get the reference point
	Point refPoint = tp.getReferencePoint(sling);
	//Calculate the tapping time according the bird type
	int dx = 0;
	int dy = 0;
	Shot shot = new Shot();
	double releaseAngle = 0;
	if (releasePoint != null) {
	    releaseAngle = tp.getReleaseAngle(sling,releasePoint);
	    System.out.println("Release Point: " + releasePoint);
	    System.out.println("Release Angle: " + Math.toDegrees(releaseAngle));

	    int tapInterval = 0;
	    int tapTime = tp.getTapTime(sling, releasePoint, targetPoint, tapInterval);
	    dx = (int)releasePoint.getX() - refPoint.x;
	    //power = (double)releasePoint.getX() - (double)refPoint.x;
	    dy = (int)releasePoint.getY() - refPoint.y;
	    shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);
	    System.out.println("shot:" + shot);
	}
	else{
	    System.err.println("No Release Point Found");
	}
	double power = tp.getReleasePower(sling, targetPoint) / 490.0 * 100.0;
	double angle = Math.toDegrees(releaseAngle);
	System.out.println("Release Angle: " + Math.toDegrees(releaseAngle));
	System.out.println("Release Power: " + power);

	System.out.println("Release dx: " + dx  + "dy:" + dy);

	return new ShootInfo((double) angle,(double) power);
        //ArrayList<Point> releasePo0ints = tp.estimateLaunchPoint(sling, targetPoint);
        //Point releasePoint = releasePoints.get(randomGenerator.nextInt(releasePoints.size()));

        //System.out.println("targetPigs.size()=" + targetPigs.size());
	//System.out.println("sling=" + sling);
	//System.out.println("targetPig_point=" + targetPoint);
	//System.out.println("angle=" + tp.getReleaseAngle(sling, releasePoint));
	//System.out.println("power=" + tp.getReleasePower(sling, releasePoint));

        //return new ShootInfo(tp.getReleaseAngle(sling, releasePoint),tp.getReleasePower(sling, releasePoint));
    }

    private void setSlingReady() {
        while (!isReady() || isSlingChanged()) {
            setReady();
        }
    }
}
