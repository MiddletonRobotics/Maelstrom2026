package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FifteenBallBluePaths
{
    public PathChain Start;
    public PathChain Pickup1;
    public PathChain Pickup12;
    public PathChain Gate;
    public PathChain Gate2;
    public PathChain Return1;
    public PathChain Return12;
    public PathChain GateIntake1;
    public PathChain GateIntake2;
    public PathChain GateReturn;
    public PathChain Pickup2;
    public PathChain Return2;
    public PathChain Pickup3;
    public PathChain Pickup32;
    public PathChain Return3;

    public FifteenBallBluePaths(Follower follower) {
        Start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.500, 129.000),

                                new Pose(58.300, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                .build();

        Pickup1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000),

                                new Pose(43.417, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.417, 60.000),

                                new Pose(11.000, 59.288)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 59.288),

                                new Pose(28.723, 60.731)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Gate2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.723, 60.731),

                                new Pose(15.138, 70.165)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.138, 70.165),

                                new Pose(39.048, 71.478)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(39.048, 71.478),

                                new Pose(58.300, 85.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        GateIntake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000),

                                new Pose(39.214, 71.452)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(39.214, 71.452),

                                new Pose(10.330, 59.654)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        GateIntake2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.330, 59.654),

                                new Pose(9.994, 54.130)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(100))

                .build();

        GateReturn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.994, 54.130),

                                new Pose(58.300, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))

                .build();

        Pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000),

                                new Pose(18.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 84.000),

                                new Pose(58.300, 85.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000),

                                new Pose(43.180, 36.610)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Pickup32 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.180, 36.610),

                                new Pose(13.912, 35.627)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Return3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.912, 35.627),

                                new Pose(57.000, 107.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .setReversed()
                .build();
    }
}
