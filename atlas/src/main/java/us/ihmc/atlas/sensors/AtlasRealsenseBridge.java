package us.ihmc.atlas.sensors;

import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class AtlasRealsenseBridge
{
   public AtlasRealsenseBridge()
   {
      RosMainNode rosMainNode = new RosMainNode(NetworkParameters.getROSURI(), "networkProcessor/realsense", true);
      rosMainNode.attachSubscriber("cam_2/depth/color/points", new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            LogTools.info("Received message: ");
         }
      });

      // link cam_2_link transform

      rosMainNode.execute();

      ThreadTools.join();
   }

   public static void main(String[] args)
   {
      new AtlasRealsenseBridge();
   }
}
