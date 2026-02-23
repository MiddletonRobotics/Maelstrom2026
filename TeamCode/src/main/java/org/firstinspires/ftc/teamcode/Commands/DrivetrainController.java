package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

import java.util.function.DoubleSupplier;

public class DrivetrainController extends CommandBase {
    private final Drivetrain dt;
    private final DoubleSupplier strafe;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotation;
    private final Maelstrom.Alliance color;

    public DrivetrainController(Maelstrom robot, DoubleSupplier strafe, DoubleSupplier forward,
            DoubleSupplier rotation) {
        this.dt = robot.dt;
        this.color = robot.color;
        this.strafe = strafe;
        this.forward = forward;
        this.rotation = rotation;

        addRequirements(dt);
    }

    @Override
    public void execute() {
        dt.setMovementVectors(strafe.getAsDouble(), forward.getAsDouble(), rotation.getAsDouble(), true, color);
        dt.calcDistance(color);
        dt.updateCompensatedGoal(color);
    }
}
