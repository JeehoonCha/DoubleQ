package ab.doubleQ;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.utils.StateUtil;
import ab.vision.ABObject;
import ab.vision.GameStateExtractor;
import ab.vision.Vision;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.*;

/**
 * Created by jeehoon on 27/04/2017.
 */
@SuppressWarnings("ALL")
public class DataCollectingAgent {

    private ActionRobot actionRobot;
    private Random randomGenerator;
    public int currentLevel = 1;
    public static int time_limit = 12;
    private Map<Integer,Integer> scores = new LinkedHashMap<Integer,Integer>();
    RandomTrajectorySelector trajectorySelector;
    private boolean firstShot;
    private Point prevTarget;
    // a standalone implementation of the Naive Agent
    public DataCollectingAgent() {
        actionRobot = new ActionRobot();
        trajectorySelector = new RandomTrajectorySelector();
        prevTarget = null;
        firstShot = true;
        randomGenerator = new Random();
        // --- go to the Poached Eggs episode level selection page ---
        ActionRobot.GoFromMainMenuToLevelSelection();
    }


    // run the client
    public void run() {

        actionRobot.loadLevel(currentLevel);
        while (true) {
            GameStateExtractor.GameState state = solve();
            if (state == GameStateExtractor.GameState.WON) {
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                int score = StateUtil.getScore(ActionRobot.proxy);
                if (!scores.containsKey(currentLevel)) {
                    scores.put(currentLevel, score);
                } else if(scores.get(currentLevel) < score) {
                    scores.put(currentLevel, score);
                }
                int totalScore = 0;
                for(Integer key: scores.keySet()){
                    totalScore += scores.get(key);
                    System.out.println(" Level " + key + " Score: " + scores.get(key));
                }
                System.out.println("Total Score: " + totalScore);
                actionRobot.loadLevel(++currentLevel);
                // make a new trajectory planner whenever a new level is entered
                trajectorySelector = new RandomTrajectorySelector();

                // first shot on this level, try high shot first
                firstShot = true;
            } else if (state == GameStateExtractor.GameState.LOST) {
                System.out.println("Restart");
                actionRobot.restartLevel();
            } else if (state == GameStateExtractor.GameState.LEVEL_SELECTION) {
                System.out.println("Unexpected level selection page, go to the last current level : " + currentLevel);
                actionRobot.loadLevel(currentLevel);
            } else if (state == GameStateExtractor.GameState.MAIN_MENU) {
                System.out.println("Unexpected main menu page, go to the last current level : " + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                actionRobot.loadLevel(currentLevel);
            } else if (state == GameStateExtractor.GameState.EPISODE_MENU) {
                System.out.println("Unexpected episode menu page, go to the last current level : " + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                actionRobot.loadLevel(currentLevel);
            }
        }
    }

    private double distance(Point p1, Point p2) {
        return Math
                .sqrt((double) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)
                        * (p1.y - p2.y)));
    }

    public GameStateExtractor.GameState solve()
    {
        BufferedImage screenshot = ActionRobot.doScreenShot();
        Vision vision = new Vision(screenshot);
        Rectangle sling = vision.findSlingshotMBR();

        // confirm the slingshot
        while (sling == null && actionRobot.getState() == GameStateExtractor.GameState.PLAYING) {
            System.out
                    .println("No slingshot detected. Please remove pop up or zoom out");
            ActionRobot.fullyZoomOut();
            screenshot = ActionRobot.doScreenShot();
            vision = new Vision(screenshot);
            sling = vision.findSlingshotMBR();
        }
        // get all the pigs
        java.util.List<ABObject> pigs = vision.findPigsMBR();

        GameStateExtractor.GameState state = actionRobot.getState();

        // if there is a sling, then play, otherwise just skip.
        if (sling != null && !pigs.isEmpty()) {
            Point releasePoint = null;
            Shot shot = new Shot();
            int dx,dy;
            {
                // (Jeehoon): Here we choose the action randomly
                double degreePercentage = (1 + randomGenerator.nextInt(98)) / (double) 100;   // 1 - 100 %
                double velocityPercentage = (1 + randomGenerator.nextInt(98)) / (double) 100;   // 1 - 100 %

                releasePoint = trajectorySelector.findReleasePoint(
                        sling,
                        degreePercentage * Math.PI / 2,
                        velocityPercentage);
                Point refPoint = trajectorySelector.getReferencePoint(sling);
                double releaseAngle = trajectorySelector.getReleaseAngle(sling, releasePoint);
                System.out.println(String.format("Angle: %s, Velocity: %s, RefPoint: %s, RelPoint: %s",
                        Math.toDegrees(releaseAngle),
                        velocityPercentage,
                        refPoint,
                        releasePoint));

                int tapInterval = 0;
                switch (actionRobot.getBirdTypeOnSling()) {
                    case YellowBird:
                    case WhiteBird:
                    case BlackBird:
                    case BlueBird:
                        tapInterval = 1 + randomGenerator.nextInt(99); // 1-100% of the way
                        break;
                    case RedBird:
                    default:
                        tapInterval = 0;
                        break;
                }

                int tapTime = 0;    // (Jeehoon): for test - tapTime 1
                dx = (int)releasePoint.getX() - refPoint.x;
                dy = (int)releasePoint.getY() - refPoint.y;
                shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);
            }

            // check whether the slingshot is changed. the change of the slingshot indicates a change in the scale.
            {
                ActionRobot.fullyZoomOut();
                screenshot = ActionRobot.doScreenShot();
                vision = new Vision(screenshot);
                Rectangle _sling = vision.findSlingshotMBR();
                if(_sling != null)
                {
                    double scale_diff = Math.pow((sling.width - _sling.width),2) +  Math.pow((sling.height - _sling.height),2);
                    if (!(scale_diff > 0))
                    {
                        if(dx < 0)
                        {
                            actionRobot.cshoot(shot);
                            state = actionRobot.getState();
                            if ( state == GameStateExtractor.GameState.PLAYING )
                            {
                                screenshot = ActionRobot.doScreenShot();
                                vision = new Vision(screenshot);
                                java.util.List<Point> traj = vision.findTrajPoints();
                                trajectorySelector.adjustTrajectory(traj, sling, releasePoint);
                                firstShot = false;
                            }
                        }
                    }
                    else
                        System.out.println("Scale is changed, can not execute the shot, will re-segement the image");
                }
                else
                    System.out.println("no sling detected, can not execute the shot, will re-segement the image");
            }

            System.out.println(String.format("Score: %s", StateUtil.getScore(ActionRobot.proxy)));

        }

        return state;
    }

    public static void main(String args[]) {

        DataCollectingAgent collectingAgent = new DataCollectingAgent();
        if (args.length > 0)
            collectingAgent.currentLevel = Integer.parseInt(args[0]);
        collectingAgent.run();

    }

}
