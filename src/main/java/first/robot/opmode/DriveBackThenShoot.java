package first.robot.opmode;

import org.wpilib.command3.Command;
import org.wpilib.opmode.PeriodicOpMode;

import first.robot.Robot;
import static org.wpilib.units.Units.Seconds;

class DriveBackThenShoot extends PeriodicOpMode {
  private final Robot robot;

  public Command autoCommand() {
    return Command.noRequirements(coroutine -> {
      robot.drivetrain.arcadeDrive(() -> -0.2, () -> 0.0).withTimeout(Seconds.of(2));
      coroutine.awaitAll(robot.intakeLauncher.shoot(), robot.feeder.feed());
    }).named("DriveBackThenShoot");
  }

  public DriveBackThenShoot(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void start() {}
}