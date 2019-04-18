package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.HashMap;
import java.util.Map;

import controller_msgs.msg.dds.DoorParameterPacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.commons.Conversions;
import us.ihmc.ros2.Ros2Node;

public class FusionSensorObjectDetectionManager
{
   private final Ros2Node ros2Node;
   private final Map<ObjectType, AbstractObjectParameterCalculator<?>> objectTypeToCalculatorMap = new HashMap<>();

   public FusionSensorObjectDetectionManager(Ros2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      defineCalculatorMap();
   }
   
   private void defineCalculatorMap()
   {
      objectTypeToCalculatorMap.put(ObjectType.Door, new DoorParameterCalculator(ros2Node, DoorParameterPacket.class));
   }
   
   public void computeAndPublish(ObjectType objectType, StereoVisionPointCloudMessage sourceMessage, RegionOfInterest roi)
   {
      long startTime = System.nanoTime();
      objectTypeToCalculatorMap.get(objectType).getPointCloudInROI(sourceMessage, roi);
      objectTypeToCalculatorMap.get(objectType).calculateAndPackResult();
      objectTypeToCalculatorMap.get(objectType).publish();
      
      long computingTime = System.nanoTime() - startTime;
      System.out.println("computing time is "+Conversions.nanosecondsToMicroseconds(computingTime));
   }
}
