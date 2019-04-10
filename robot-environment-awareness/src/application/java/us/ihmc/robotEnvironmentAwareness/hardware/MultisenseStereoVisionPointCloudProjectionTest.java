package us.ihmc.robotEnvironmentAwareness.hardware;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Random;
import java.util.Scanner;

import javax.imageio.ImageIO;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class MultisenseStereoVisionPointCloudProjectionTest extends AbstractRosTopicSubscriber<PointCloud2>
{
   private static final int MAX_NUMBER_OF_POINTS = 200000;

   private final Scanner commandScanner;
   private static final String commandToReceiveNewImage = "s";
   private int savingIndex = 0;

   public MultisenseStereoVisionPointCloudProjectionTest() throws URISyntaxException
   {
      super(PointCloud2._TYPE);
      commandScanner = new Scanner(System.in);

      URI masterURI = new URI("http://10.6.192.14:11311");
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber("/multisense/image_points2_color_world", this);

      rosMainNode.execute();
   }

   @Override
   public void onNewMessage(PointCloud2 cloudHolder)
   {
      String command = commandScanner.next();
      if (command.contains(commandToReceiveNewImage))
      {
         int height = 544;
         int width = 1024;
         BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

         UnpackedPointCloud pointCloudData = RosPointCloudSubscriber.unpackPointsAndIntensities(cloudHolder);

         Point3D[] pointCloud = pointCloudData.getPoints();
         Color[] colors = pointCloudData.getPointColors();

         Random random = new Random();
         int numberOfPoints = pointCloud.length;

         while (numberOfPoints > MAX_NUMBER_OF_POINTS)
         {
            int indexToRemove = random.nextInt(numberOfPoints);
            int lastIndex = numberOfPoints - 1;

            pointCloud[indexToRemove] = pointCloud[lastIndex];
            colors[indexToRemove] = colors[lastIndex];

            numberOfPoints--;
         }

         // K: simplest, more accurate.
         double fx = 601.5020141601562;
         double fy = 602.0339965820312;
         double cx = 520.92041015625;
         double cy = 273.5399169921875;

         // P: bad. additional equation(rational radial model) we need. but no information.
         //         double fx = 584.234619140625;
         //         double fy = 584.234619140625;
         //         double cx = 512.0;
         //         double cy = 272.0;

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];

            double cameraX = -scanPoint.getY();
            double cameraY = -scanPoint.getZ();
            double cameraZ = scanPoint.getX();

            double normX = cameraX / cameraZ;
            double normY = cameraY / cameraZ;

            int u = (int) (fx * normX + cx);
            int v = (int) (fy * normY + cy);

            boolean inImage = false;
            if (u >= 0 && u < width)
               if (v >= 0 && v < height)
                  inImage = true;

            if (inImage)
               bufferedImage.setRGB(u, v, colors[i].getRGB());
         }

         File outputfile = new File("projected_image_" + savingIndex + ".png");
         try
         {
            ImageIO.write(bufferedImage, "png", outputfile);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         savingIndex++;
      }

   }

   public static void main(String[] args) throws URISyntaxException
   {
      new MultisenseStereoVisionPointCloudProjectionTest();
   }
}
