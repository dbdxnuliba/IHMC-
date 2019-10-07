package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import java.util.Random;

import controller_msgs.msg.dds.DepthCloudMessage;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber.UnpackedPointCloud;

public class PointCloudData
{
   protected final long timestamp;
   protected int numberOfPoints;
   protected final Point3D[] pointCloud;
   
   public PointCloudData(long timestamp, Point3D[] scanPoints)
   {
      this.timestamp = timestamp;

      this.pointCloud = scanPoints;
      this.numberOfPoints = scanPoints.length;
   }

   public PointCloudData(PointCloud2 rosPointCloud2, int maxSize)
   {
      timestamp = rosPointCloud2.getHeader().getStamp().totalNsecs();

      UnpackedPointCloud unpackPointsAndIntensities = RosPointCloudSubscriber.unpackPointsAndIntensities(rosPointCloud2);
      pointCloud = unpackPointsAndIntensities.getPoints();

      if (unpackPointsAndIntensities.getPoints().length <= maxSize)
      {
         numberOfPoints = pointCloud.length;
      }
      else
      {
         Random random = new Random();
         int currentSize = pointCloud.length;

         while (currentSize > maxSize)
         {
            int nextToRemove = random.nextInt(currentSize);
            pointCloud[nextToRemove] = pointCloud[currentSize - 1];
            pointCloud[currentSize - 1] = null;

            currentSize--;
         }
         numberOfPoints = maxSize;
      }
   }

   public void removePoints(Point3DBasics min, Point3DBasics max)
   {
      boolean remove = false;
      for (int i = 0; i < numberOfPoints; i++)
      {
         remove = false;
         for (int j = 0; j < 3; j++)
         {
            if (min.getElement(j) > pointCloud[i].getElement(j) || max.getElement(j) < pointCloud[i].getElement(j))
            {
               remove = true;
               break;
            }
         }

         if (remove)
         {
            pointCloud[i] = pointCloud[numberOfPoints - 1];
            pointCloud[numberOfPoints - 1] = null;

            i--;
            numberOfPoints--;
         }
      }
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public DepthCloudMessage toDepthCloudMessage()
   {
      long timestamp = this.timestamp;
      float[] pointCloudBuffer = new float[3 * numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();
      }

      return MessageTools.createDepthCloudMessage(timestamp, pointCloudBuffer);
   }
   
   public void applyTransform(RigidBodyTransform transform)
   {
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i].applyTransform(transform);
      }
   }
}
