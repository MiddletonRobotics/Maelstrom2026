package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FifteenFarRedSpikePaths
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
    public PathChain Return3;

    public PathChain AltPickup;
    public PathChain AltReturn;

    public PathChain Leave;

    public FifteenFarRedSpikePaths(Follower follower) {
        Start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 9.000).mirror(),

                                new Pose(56.000, 18.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        //First HP Cycle
        Pickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000).mirror(),

                                new Pose(15.574, 9.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.574, 9.000).mirror(),

                                new Pose(8.500, 8.500).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 8.500).mirror(),

                                new Pose(15.490, 8.500).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.490, 8.500).mirror(),

                                new Pose(8.500, 8.500).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 8.500).mirror(),

                                new Pose(56.000, 18.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        //Spike Mark
        Pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000).mirror(),

                                new Pose(46.371, 35.591).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup22 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.371, 35.591).mirror(),

                                new Pose(7.800, 36.102).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.800, 36.102).mirror(),

                                new Pose(56.000, 18.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        //Gate Release Cycle
        Pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000).mirror(),

                                new Pose(8.500, 10.700).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.500, 10.700).mirror(),

                                new Pose(56.000, 18.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        //Alternate Cycle
        AltPickup = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000).mirror(),

                                new Pose(12.756, 29.198).mirror()
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(12.756, 29.198).mirror(),

                                new Pose(9.620, 51.157).mirror()
                        )
                ).setTangentHeadingInterpolation()

                .build();

        AltReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.620, 51.157).mirror(),

                                new Pose(56.000, 18.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        //Leave
        Leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 18.000).mirror(),

                                new Pose(43.000, 18.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }
}
