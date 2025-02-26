package frc.robot.data;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotControlState;

public class RobotControlStateSendable implements Sendable {
        @Override
        public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("RobotControlData");

                builder.addStringProperty("side",
                                () -> RobotControlState.getSide().toString(),
                                null);

                builder.addStringProperty("zone",
                                () -> RobotControlState.getAllianceZone(),
                                null);

                builder.addStringProperty("alliance",
                                () -> RobotControlState.getAlliance(),
                                null);
        }
}
