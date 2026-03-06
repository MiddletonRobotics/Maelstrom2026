package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TwelveBallRedLine {
    public PathChain Start;
    public PathChain Pickup1;
    public PathChain Pickup12;
    public PathChain Gate;
    public PathChain Gate2;
    public PathChain Return1;
    public PathChain Return12;
    public PathChain Pickup2;
    public PathChain Return2;
    public PathChain Pickup3;
    public PathChain Pickup32;
    public PathChain Return3;

    public TwelveBallRedLine(Follower follower) {
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

        Gate = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 59.288).mirror(),

                                new Pose(28.723, 60.731).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Gate2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.723, 60.731).mirror(),

                                new Pose(15.138, 70.165).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.138, 70.165).mirror(),

                                new Pose(40.048, 71.478).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.048, 71.478).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(18.000, 84.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.000, 84.000).mirror(),

                                new Pose(58.300, 85.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.300, 85.000).mirror(),

                                new Pose(45.180, 36.610).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Pickup32 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.180, 36.610).mirror(),

                                new Pose(11, 35.627).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Return3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11, 35.627).mirror(),

                                new Pose(57.000, 111.000).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
}


