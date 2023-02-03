package frc.robot.drive;

import java.util.ArrayList;

public class DriveNodePoints {

    // X is index 0, Y is index 1
    private ArrayList<Double> vectorLeftNode = new ArrayList<Double>();
    private ArrayList<Double> vectorMiddleNode = new ArrayList<Double>();
    private ArrayList<Double> vectorRightNode = new ArrayList<Double>();

    private double xToAprilTag;
    private double yToAprilTag;
    private double nodeWidthIn;

    public DriveNodePoints (double xToATag, double yToATag) {
        xToAprilTag = xToATag;
        yToAprilTag = yToATag;
        nodeWidthIn = 18.25;
    }

    public void updatePoints (double xToATag, double yToATag) {
        xToAprilTag = xToATag;
        yToAprilTag = yToATag;
    }

    public void calculateVectorLeftNode() {
        vectorLeftNode.set(0, xToAprilTag - nodeWidthIn);
        vectorLeftNode.set(1, yToAprilTag);
    }

    public void calculateVectorMiddleNode() {
        vectorMiddleNode.set(0, xToAprilTag);
        vectorMiddleNode.set(1, yToAprilTag);
    }

    public void calculateVectorRightNode() {
        vectorRightNode.set(0, xToAprilTag + nodeWidthIn);
        vectorRightNode.set(1, yToAprilTag);
    }
}
