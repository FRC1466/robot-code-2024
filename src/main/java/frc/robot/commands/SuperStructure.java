package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Manipulator.EndEffector;
import frc.robot.subsystems.Manipulator.Indexer;
import frc.robot.subsystems.Manipulator.Intake;
import frc.robot.subsystems.Manipulator.Dragonhead;

public class SuperStructure {
  private final EndEffector effector;
  private final Dragonhead arm;
  private Intake intake;
  private Indexer index;

  public SuperStructure(EndEffector effector, Dragonhead arm, Intake intake, Indexer index) {
    this.effector = effector;
    this.arm = arm;
    this.intake = intake;
    this.index = index;
  }

  public Command pickupGround() {
    return intake.intake().alongWith(index.outtake());
  }
  public Command subWooferShoot(){
    return arm.store().andThen(index.outtake().alongWith(effector.shoot()));
}
public Command podiumShoot(){
    return arm.podium().andThen(index.outtake()).alongWith(effector.shoot());
}
public Command ampShoot(){
    return arm.amp().andThen(index.outtake()).alongWith(effector.ampShoot());
}

}
