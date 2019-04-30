package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import controller_msgs.msg.dds.DoorParameterPacket;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.ros2.Ros2Node;

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   public DoorParameterCalculator(Ros2Node ros2Node, Class<DoorParameterPacket> packetType)
   {
      super(ros2Node, packetType);
   }

   /**
    * ROI(Door), ROI(Door Handle)
    */
   @Override
   public void calculate(RegionOfInterest... additionalROIs)
   {
      if(additionalROIs == null)
      {
         // TODO : assume the handle located on left side of the image.
      }
      else
      {
         
      }
   }
}