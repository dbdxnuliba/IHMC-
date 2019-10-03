package us.ihmc.atlas.sensors;

import sensor_msgs.PointCloud2;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class AtlasRealsensePublisher
{
   public AtlasRealsensePublisher()
   {
      RosMainNode rosMainNode = new RosMainNode(NetworkParameters.getROSURI(), "networkProcessor/realsense", true);
      rosMainNode.attachSubscriber(AtlasSensorInformation.realsenseTopicName, new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(PointCloud2 pointCloud)
         {
            LogTools.info("Received message: ");
         }
      });

      // subscribe to humanoid reference frames


      // start thread


      rosMainNode.execute();

      ThreadTools.join();
   }

   private void run()
   {
      // latest point cloud 2
      // latest reference frames


   }

   public static void main(String[] args)
   {
      new AtlasRealsensePublisher();
   }
}
