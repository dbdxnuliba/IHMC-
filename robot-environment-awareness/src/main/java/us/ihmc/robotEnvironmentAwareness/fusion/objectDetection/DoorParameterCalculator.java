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
      int size = pointCloudToCalculate.size();
      double xtotal = 0 , ytotal = 0, ztotal = 0;
      for (int i = 0; i < size; i++)
      {
         xtotal+= pointCloudToCalculate.get(i).getX();
         ytotal+= pointCloudToCalculate.get(i).getY();
         ztotal+= pointCloudToCalculate.get(i).getZ();
      }
      System.out.println("door center " +xtotal/size +" "+ ytotal/size +" "+ztotal/size);
   }
}
