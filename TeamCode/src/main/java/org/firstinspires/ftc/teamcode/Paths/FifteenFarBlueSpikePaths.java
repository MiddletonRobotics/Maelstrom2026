package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FifteenFarBlueSpikePaths
{
    public PathChain Start;
    public PathChain Pickup1;
    public PathChain Pickup12;
    public PathChain Pickup13;
    public PathChain Pickup14;
    public PathChain Return1;
    public PathChain Pickup2;
    public PathChain Pickup22;
    public PathChain Return2;
    public PathChain Pickup3;
    public PathChain Pickup31;
    public PathChain Pickup32;
    public PathChain Return3;

    public PathChain Leave;

    public FifteenFarBlueSpikePaths(Follower follower) {
        Start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 9.000),

                                new Pose(56.000, 18.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        //First HP Cycle
        Pickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000),

                                new Pose(15.574, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.574, 9.000),

                                new Pose(8.500, 8.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 8.500),

                                new Pose(15.490, 8.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.490, 8.500),

                                new Pose(8.500, 8.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 8.500),

                                new Pose(56.000, 18.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        //Spike Mark
        Pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000),

                                new Pose(46.371, 35.591)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup22 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.371, 35.591),

                                new Pose(7.800, 36.102)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.800, 36.102),

                                new Pose(56.000, 18.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        //Gate Release Cycle
        Pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000),

                                new Pose(8.500, 10.700)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup31 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 10.700),

                                new Pose(21.000, 11.700)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(21.000, 11.700),

                                new Pose(8.500, 14.600)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 14.600),

                                new Pose(56.000, 18.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        //Leave
        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000),

                                new Pose(43.000, 18.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

}
