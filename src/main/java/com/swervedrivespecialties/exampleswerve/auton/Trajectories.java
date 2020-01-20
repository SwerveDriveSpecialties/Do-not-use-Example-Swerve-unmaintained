package com.swervedrivespecialties.exampleswerve.auton;

import java.util.function.Supplier;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Trajectories {
    ////////// UNIVERSAL TRAJECTORY CONSTANTS /////////////
    private static final int kSubdivideIterations = 8;
    private static final double kDefaultMaxSpeed = 10 * 12;
    private static final double kMaxAccel = 13. * 12;
    private static final double kMaxCentripedalAccel = 25. * 12;

    ////////////// TEST TRAJECTORY ////////////////////////
    private static Trajectory testTrajectory;
    private static final double testTrajectoryMaxVelo = 6 * 12;
    private static final ITrajectoryConstraint[] testTrajectoryConstraints = {new MaxVelocityConstraint(testTrajectoryMaxVelo), 
                                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
    private static final double testTrajectoryEndVelo = 3 * 12;
    private static final Rotation2 testPathRotation = Rotation2.fromDegrees(-90.);
    private static final Rotation2 testPathStartRotation = Rotation2.ZERO;

    private static void generateTestTrajectory(){
        Path testPath = new Path(testPathStartRotation);
        testPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(24, 0)), testPathRotation);
        //testPath.addSegment(new PathArcSegment(new Vector2(24, 0), new Vector2(72, -48), new Vector2(24, -48)), testPathRotation);
        testPath.subdivide(kSubdivideIterations);
        testTrajectory = new Trajectory(0.0, testTrajectoryEndVelo, testPath, testTrajectoryConstraints);
    }

    private static Trajectory getTestTrajectory(){
        return testTrajectory;
    }

    public static Supplier<Trajectory> testTrajectorySupplier = () -> getTestTrajectory();
    ////////////////////////////////////////////////////////


    //////////////// EXAMPLE TRAJECTORY //////////////////
    private static Trajectory exampleTrajectory;
    private static final double exampleTrajectoryX = 42;
    private static final double exampleTrajectoryY = -24;
    private static final double exampleTrajectoryPhi = -60; //degrees

    private static void generateExampleTrajectory(){
        exampleTrajectory = generateLineTrajectory(new Vector2(exampleTrajectoryX, exampleTrajectoryY), Rotation2.fromDegrees(exampleTrajectoryPhi));
    }
    
    private static Trajectory getExampleTrajectory(){
        return exampleTrajectory;
    }

    public static Supplier<Trajectory> exampleTrajectorySupplier = () -> getExampleTrajectory();
    //////////////////////////////////////////////////////


    public static void generateAllTrajectories(){
        generateTestTrajectory();
        generateExampleTrajectory();
    }


    /////////////////// Helper Methods /////////////////////////

    private static Trajectory generateLineTrajectory(Vector2 line, Rotation2 endRotation){
        ITrajectoryConstraint[] lineTrajectoryConstraints = {new MaxVelocityConstraint(kDefaultMaxSpeed), 
                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        Path linePath = new Path(Rotation2.ZERO);
        linePath.addSegment(new PathLineSegment(Vector2.ZERO, line), endRotation);
        linePath.subdivide(kSubdivideIterations);
        Trajectory resultTrajectory = new Trajectory(0.0, 0.0, linePath, lineTrajectoryConstraints);
        return resultTrajectory;
    }
}