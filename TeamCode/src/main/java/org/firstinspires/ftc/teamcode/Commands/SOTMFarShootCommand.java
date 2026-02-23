package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

/**
 * Shoot-on-the-move far variant.
 * State-machine command so execute() runs every loop, continuously
 * updating compensated distance and flywheel velocity.
 * Uses 0.7 intake power instead of full spinIn().
 */
public class SOTMFarShootCommand extends CommandBase {
    private final Intake intake;
    private final Shooter shooter;
    private final Drivetrain dt;

    private enum State {
        STOP_INTAKE, SPIN_UP, KICKER_UP, WAIT_KICKER, SLOW_IN, WAIT_BALL,
        WAIT_SETTLE, GATE_UP, CLEANUP, DONE
    }

    private State state = State.STOP_INTAKE;
    private final ElapsedTime timer = new ElapsedTime();

    public SOTMFarShootCommand(Maelstrom robot) {
        this.intake = robot.intake;
        this.shooter = robot.shooter;
        this.dt = robot.dt;

        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        state = State.STOP_INTAKE;
        timer.reset();
    }

    @Override
    public void execute() {
        // ── Continuous SOTM updates ──
        shooter.updateDistance(dt.compensatedDistance);
        shooter.shootAutoVelocity();

        // ── State machine (mirrors TeleOPFarShootCommand sequence) ──
        switch (state) {
            case STOP_INTAKE:
                intake.stop();
                state = State.SPIN_UP;
                break;

            case SPIN_UP:
                // shootAutoVelocity already called above
                state = State.KICKER_UP;
                break;

            case KICKER_UP:
                if (shooter.atSpeed()) {
                    intake.kickerUp();
                    timer.reset();
                    state = State.WAIT_KICKER;
                }
                break;

            case WAIT_KICKER:
                if (timer.milliseconds() >= 200) {
                    intake.setPower(0.7); // Far variant: reduced intake power
                    timer.reset();
                    state = State.SLOW_IN;
                }
                break;

            case SLOW_IN:
                if (timer.milliseconds() >= 500) {
                    timer.reset();
                    state = State.WAIT_BALL;
                }
                break;

            case WAIT_BALL:
                if (intake.ballReady() || timer.milliseconds() >= 300) {
                    timer.reset();
                    state = State.WAIT_SETTLE;
                }
                break;

            case WAIT_SETTLE:
                if (timer.milliseconds() >= 50) {
                    intake.kicker2Up();
                    timer.reset();
                    state = State.GATE_UP;
                }
                break;

            case GATE_UP:
                if (timer.milliseconds() >= 200) {
                    state = State.CLEANUP;
                }
                break;

            case CLEANUP:
                intake.kicker2down();
                intake.stop();
                intake.kickerDown();
                state = State.DONE;
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }
}
