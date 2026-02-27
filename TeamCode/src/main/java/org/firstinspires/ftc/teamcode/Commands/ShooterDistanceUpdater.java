package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class ShooterDistanceUpdater extends CommandBase {
    private final Shooter shooter;
    private final Drivetrain dt;

    public ShooterDistanceUpdater(Maelstrom robot) {
        this.shooter = robot.shooter;
        this.dt = robot.dt;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.updateDistance(dt.compensatedDistance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
