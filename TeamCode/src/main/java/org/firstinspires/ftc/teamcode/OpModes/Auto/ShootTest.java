package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Commands.FarShootCommand;
import org.firstinspires.ftc.teamcode.Commands.FinalShootCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommandV2;
import org.firstinspires.ftc.teamcode.Commands.ShootWithSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

@Autonomous (name="ShootTest")
public class ShootTest extends CommandOpMode
{
    public Maelstrom robot;

    @Override
    public void initialize()
    {
        robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.BLUE,gamepad1,gamepad2);
        robot.shooter.setTargetVelocity(1550);

        schedule(
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> robot.shooter.enableFlywheel()),
                        new WaitCommand(2000),
                        new ShootCommandV2(robot)
                )
        );
    }
}
