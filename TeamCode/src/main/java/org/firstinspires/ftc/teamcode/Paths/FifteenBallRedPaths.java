package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FifteenBallRedPaths
{
    public PathChain Start;
    public PathChain Pickup1;
    public PathChain Pickup12;
    public PathChain Return1;
    public PathChain GateIntake1;
    public PathChain GateIntake2;
    public PathChain GateReturn;
    public PathChain Pickup2;
    public PathChain Return2;
    public PathChain Pickup3;
    public PathChain Pickup32;
    public PathChain Return3;

    public FifteenBallRedPaths(Follower follower) {
        Start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.500, 129.000).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();

        Pickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(43.417, 60.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.417, 60.000).mirror(),

                                new Pose(11.000, 59.288).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();


        Return1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 59.288).mirror(),

                                new Pose(39.048, 60.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(39.048, 60.000).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        GateIntake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(39.214, 71.452).mirror()
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(39.214, 71.452).mirror(),

                                new Pose(10.330, 59.654).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        GateIntake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.330, 59.654).mirror(),

                                new Pose(9.994, 54.130).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(80))

                .build();

        GateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.994, 54.130).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(0))

                .build();

        Pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(16.000, 84.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 84.000).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(43.180, 36.610).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup32 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.180, 36.610).mirror(),

                                new Pose(13.912, 35.627).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.912, 35.627).mirror(),

                                new Pose(57.000, 107.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}
