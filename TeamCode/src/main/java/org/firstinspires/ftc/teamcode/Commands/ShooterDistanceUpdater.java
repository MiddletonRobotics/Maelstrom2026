package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ShooterDistanceUpdater extends CommandBase
{
    private final Shooter shooter;
    private DoubleSupplier dist;

    public ShooterDistanceUpdater(Maelstrom robot, DoubleSupplier distance)
    {
        this.shooter=robot.shooter;
        this.dist=distance;

        addRequirements(shooter);
    }

    @Override
    public void execute()
    {
        shooter.updateDistance(dist.getAsDouble());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
