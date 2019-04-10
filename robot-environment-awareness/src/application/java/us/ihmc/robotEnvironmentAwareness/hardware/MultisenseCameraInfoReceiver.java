package us.ihmc.robotEnvironmentAwareness.hardware;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import sensor_msgs.CameraInfo;
import std_msgs.Header;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class MultisenseCameraInfoReceiver extends AbstractRosTopicSubscriber<CameraInfo>
{
   private boolean showInfo = true;

   public MultisenseCameraInfoReceiver() throws URISyntaxException, IOException
   {
      super(CameraInfo._TYPE);
      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "CameraInfoPublisher", true);
      rosMainNode.attachSubscriber("/multisense/left/camera_info", this);
      rosMainNode.execute();
   }

   @Override
   public void onNewMessage(CameraInfo cameraInfo)
   {
      if (!showInfo)
         return;

      System.out.println("## received");
      String distortionModel = cameraInfo.getDistortionModel();
      System.out.println(distortionModel);
      double[] d = cameraInfo.getD();
      double[] k = cameraInfo.getK();
      double[] p = cameraInfo.getP();
      double[] r = cameraInfo.getR();
      Header header = cameraInfo.getHeader();
      int height = cameraInfo.getHeight();
      int width = cameraInfo.getWidth();

      System.out.println("D");
      for (int i = 0; i < d.length; i++)
         System.out.println(d[i]);

      System.out.println("K");
      for (int i = 0; i < k.length; i++)
         System.out.println(k[i]);

      System.out.println("P");
      for (int i = 0; i < p.length; i++)
         System.out.println(p[i]);

      System.out.println("R");
      for (int i = 0; i < r.length; i++)
         System.out.println(r[i]);

      System.out.println(header.getStamp() + " " + header.getFrameId());

      System.out.println("height = " + height + " width = " + width);

      showInfo = false;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new MultisenseCameraInfoReceiver();
   }
}
