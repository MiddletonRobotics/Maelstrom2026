package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

/**
 * Shoot-on-the-move far variant.
 * Wraps TeleOPFarShootCommand as the deadline, with a RunCommand
 * that continuously updates compensated distance and flywheel velocity.
 */
public class SOTMFarShootCommand extends ParallelDeadlineGroup {
    public SOTMFarShootCommand(Maelstrom robot) {
        super(new TeleOPFarShootCommand(robot),new RunCommand(() -> robot.shooter.updateCommand(robot.dt.compensatedDistance)));
        addRequirements(robot.intake, robot.shooter);
    }
}
