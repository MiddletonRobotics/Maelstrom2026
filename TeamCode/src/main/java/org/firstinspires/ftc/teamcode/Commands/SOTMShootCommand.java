package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

/**
 * Shoot-on-the-move close variant.
 * Wraps TeleOPShootCommand as the deadline, with a RunCommand
 * that continuously updates compensated distance and flywheel velocity.
 */
public class SOTMShootCommand extends ParallelDeadlineGroup {
    public SOTMShootCommand(Maelstrom robot) {
        super(new TeleOPShootCommand(robot),new RunCommand(() -> robot.shooter.updateCommand(robot.dt.compensatedDistance)));
        addRequirements(robot.intake, robot.shooter);
    }
}
