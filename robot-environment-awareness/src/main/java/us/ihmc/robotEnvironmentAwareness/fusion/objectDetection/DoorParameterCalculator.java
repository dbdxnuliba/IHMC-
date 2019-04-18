package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import controller_msgs.msg.dds.DoorParameterPacket;
import us.ihmc.ros2.Ros2Node;

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   public DoorParameterCalculator(Ros2Node ros2Node, Class<DoorParameterPacket> packetType)
   {
      super(ros2Node, packetType);
   }

   @Override
   public void calculateAndPackResult()
   {
      // TODO : pack result on newPacket.

   }
}
