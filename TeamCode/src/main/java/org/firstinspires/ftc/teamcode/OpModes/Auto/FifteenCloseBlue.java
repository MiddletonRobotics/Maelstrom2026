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

import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.ShootCommandV2;
import org.firstinspires.ftc.teamcode.Paths.FifteenBallBluePaths;
import org.firstinspires.ftc.teamcode.Paths.TwelveBallBlueLine;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
@Autonomous(name="15CloseBlue")
public class FifteenCloseBlue extends CommandOpMode
{
    private Maelstrom robot;
    private FifteenBallBluePaths paths;
    private Follower follower;


    @Override
    public void initialize()
    {
        robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.BLUE,gamepad1,gamepad2);
        follower=robot.dt.follower;
        follower.setStartingPose(new Pose(25.5,129,Math.toRadians(143)));
        robot.shooter.shootMid();
        paths= new FifteenBallBluePaths(follower);

        schedule(
                new WaitUntilCommand(this::opModeIsActive),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.shooter.setHood(0.15)),
                                //new InstantCommand(() -> robot.turret.setTempOffset(-47)),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.shooter.enableFlywheel()),
                                        new InstantCommand(() -> robot.turret.setPointMode()),
                                        new InstantCommand(() -> robot.turret.setManualAngle(-47)),
                                        new FollowPathCommand(follower,paths.Start,true)
                                ),
                                new WaitCommand(50),
                                new ShootCommandV2(robot),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPathCommand(follower,paths.Pickup1,true,0.9),
                                new FollowPathCommand(follower,paths.Pickup12,false,0.9),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPathCommand(follower,paths.Return1,false),
                                //new InstantCommand(robot.intake::stop),
                                new WaitCommand(50),
                                new ShootCommandV2(robot),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPathCommand(follower,paths.GateIntake1,true).withTimeout(2500),
                                new FollowPathCommand(follower,paths.GateIntake2,true),
                                new FollowPathCommand(follower,paths.GateReturn,true),
                                new ShootCommandV2(robot),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPath(robot,paths.Pickup2,true,0.9).withTimeout(2500),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPathCommand(follower,paths.Return2),
                                //new InstantCommand(() -> robot.intake.stop()),
                                new ShootCommandV2(robot),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new FollowPath(robot,paths.Pickup3,false,0.9),
                                new FollowPath(robot,paths.Pickup32,true,0.9).withTimeout(3000),
                                new WaitCommand(250),
                                new InstantCommand(() -> robot.intake.spinIn()),
                                new InstantCommand(() -> robot.turret.setManualAngle(-30)),
                                new FollowPathCommand(follower,paths.Return3,true),
                                new ShootCommandV2(robot),
                                new ParallelCommandGroup(
                                        new InstantCommand(()->robot.turret.setManualAngle(0)),
                                        new InstantCommand(() -> robot.shooter.stopFlywheel()),
                                        new InstantCommand(() -> robot.reset())
                                )
                        )
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
