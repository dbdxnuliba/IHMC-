package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DoorParameterPacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.Ros2Node;

public class FusionSensorObjectDetectionManager
{
   private final AbstractObjectParameterCalculator<DoorParameterPacket> doorParameterCalculator;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);

   public FusionSensorObjectDetectionManager(Ros2Node ros2Node)
   {
      doorParameterCalculator = new DoorParameterCalculator(ros2Node, DoorParameterPacket.class);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   /**
    * To calculate {@code DoorParameterPacket}, calculator needs handle ROI with door ROI.
    * @param doorROI
    * @param handleROI
    */
   public void calculateDoorParameter(RegionOfInterest doorROI, RegionOfInterest handleROI)
   {
      long startTime = System.nanoTime();
      doorParameterCalculator.initialize();
      doorParameterCalculator.trimPointCloudInROI(latestStereoVisionPointCloudMessage.get(), doorROI);
      doorParameterCalculator.calculate(handleROI);
      doorParameterCalculator.publish();
      long computingTime = System.nanoTime() - startTime;
      LogTools.info("computing time is " + Conversions.nanosecondsToMicroseconds(computingTime));
   }
}
