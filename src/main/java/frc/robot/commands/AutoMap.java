package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator.Dragonhead;
import frc.robot.subsystems.Manipulator.EndEffector;
import java.util.HashMap;

public class AutoMap {
  public final HashMap<String, Command> eventMap = new HashMap<>();

  public AutoMap(SuperStructure superstructure, EndEffector effector, Dragonhead arm) {

    eventMap.put("Pickup", superstructure.pickupGround());
    eventMap.put("SpeakerShoot", superstructure.subWooferShoot());
    eventMap.put("PodiumShoot", superstructure.podiumShoot());
    eventMap.put("AmpShoot", superstructure.ampShoot());
    eventMap.put("StoreArm", arm.store());
  }

  public HashMap<String, Command> getEventMap() {
    return eventMap;
  }
}
