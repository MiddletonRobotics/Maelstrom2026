package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class TeleOPShootCommand extends SequentialCommandGroup {
    private Intake intake;
    private Shooter shooter;

    public TeleOPShootCommand(Maelstrom robot) {
        intake = robot.intake;
        shooter = robot.shooter;
        addCommands(
                // new InstantCommand(intake::slowSpinOut),
                // new WaitCommand(50),
                new InstantCommand(shooter::enableShooting),
                new InstantCommand(intake::stop),
                new InstantCommand(shooter::shootAutoVelocity),
                new WaitUntilCommand(shooter::atSpeed).withTimeout(500),
                new InstantCommand(intake::kickerUp),
                new WaitCommand(200),
                new InstantCommand(intake::spinIn),
                new WaitUntilCommand(shooter::speedDropped).withTimeout(1000),
                new WaitCommand(200),
                new InstantCommand(intake::kicker2Up),
                new WaitCommand(200),
                new InstantCommand(intake::kicker2down),
                new InstantCommand(intake::stop),
                new InstantCommand(intake::kickerDown),
                new InstantCommand(shooter::disableShooting)
        );
        addRequirements(intake,shooter);
    }
}
