package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

import java.util.function.Supplier;

public class ParkCommand extends CommandBase
{
    private final Drivetrain dt;
    private final Pose parkPose;
    private final Supplier<Pose> currentPose;
    private final Maelstrom.Alliance color;

    public ParkCommand(Maelstrom robot, Supplier<Pose> currentPose, Pose target)
    {
        this.dt=robot.dt;
        this.currentPose=currentPose;
        this.color=robot.color;
        parkPose=target;


        addRequirements(dt);
    }

    @Override
    public void initialize()
    {
        if(color.equals(Maelstrom.Alliance.BLUE))
        {
            PathChain bluePark= dt.follower.pathBuilder().addPath(new BezierLine(currentPose.get(),parkPose)).setLinearHeadingInterpolation(currentPose.get().getHeading(), parkPose.getHeading()).build();
            dt.follower.followPath(bluePark,true);
        }
        else if(color.equals(Maelstrom.Alliance.RED))
        {
            PathChain redPark= dt.follower.pathBuilder().addPath(new BezierLine(currentPose.get(),parkPose.mirror())).setLinearHeadingInterpolation(dt.follower.getHeading(), parkPose.mirror().getHeading()).build();
            dt.follower.followPath(redPark,true);
        }
    }

    @Override
    public boolean isFinished()
    {
        return dt.follower.atParametricEnd();
    }

    @Override
    public void end(boolean interrupted)
    {
        dt.follower.breakFollowing();
    }

}
