package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.FarShootCommand;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.ShootCommandV2;
import org.firstinspires.ftc.teamcode.Paths.FifteenFarRedSpikePaths;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

@Autonomous(name="15FarRed")
public class FifteenFarRed extends CommandOpMode
{
    private Maelstrom robot;
    private Follower follower;
    private FifteenFarRedSpikePaths paths;

    @Override
    public void initialize()
    {
        robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.RED,gamepad1,gamepad2);
        follower=robot.dt.follower;
        follower.setStartingPose(new Pose(56,9,Math.toRadians(180)).mirror());
        robot.shooter.setTargetVelocity(2025);
        robot.shooter.useAuto=false;
        robot.shooter.setHood(0.3);
        paths= new FifteenFarRedSpikePaths(follower);

        schedule(
                new WaitUntilCommand(this::opModeIsActive),
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.shooter.setHood(0.3)),
                        new InstantCommand(() -> robot.turret.setTempOffset(67)),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.shooter.enableFlywheel()),
                                new InstantCommand(() -> robot.turret.setPointMode()),
                                new InstantCommand(() -> robot.turret.setManualAngle(68)),
                                new FollowPathCommand(follower,paths.Start,true)
                        ),
                        new WaitCommand(600),
                        new ShootCommandV2(robot),
                        new InstantCommand(() -> robot.intake.spinIn()),
                        new FollowPath(robot,paths.Pickup1,true,1).withTimeout(2500),
                        new FollowPathCommand(follower,paths.Pickup12,false).withTimeout(1000),
                        new WaitCommand(500),
                        new FollowPathCommand(follower,paths.Pickup13,false),
                        new FollowPathCommand(follower,paths.Pickup14,false).withTimeout(1000),
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.intake.idle()),
                        new FollowPathCommand(follower,paths.Return1),
                        new InstantCommand(() -> robot.intake.stop()),
                        new ShootCommandV2(robot),
                        new InstantCommand(robot.intake::spinIn),
                        new FollowPathCommand(follower,paths.Pickup3,true).withTimeout(2500),
                        new InstantCommand(robot.intake::idle),
                        new FollowPathCommand(follower,paths.Return3,true),
                        new ShootCommandV2(robot),
                        new InstantCommand(robot.intake::spinIn),
                        new FollowPathCommand(follower,paths.Pickup3,true).withTimeout(2500),
                        new InstantCommand(robot.intake::idle),
                        new FollowPathCommand(follower,paths.Return3,true),
                        new ShootCommandV2(robot),
                        new InstantCommand(robot.intake::spinIn),
                        new FollowPathCommand(follower,paths.Pickup3,true).withTimeout(2500),
                        new InstantCommand(robot.intake::idle),
                        new FollowPathCommand(follower,paths.Return3,true),
                        new ShootCommandV2(robot),
                        new FollowPathCommand(follower,paths.Leave),
                        new InstantCommand(() -> robot.shooter.stopFlywheel()),
                        new InstantCommand(() -> robot.turret.setManualAngle(0)),
                        new InstantCommand(() -> robot.reset())
                )
        );
    }

    @Override
    public void end()
    {
        if(robot!=null)
        {
            for (int i=0; i<150; i++)
            {
                robot.dt.follower.update();
            }
            Drivetrain.startPose=robot.dt.follower.getPose();
        }
    }
}
