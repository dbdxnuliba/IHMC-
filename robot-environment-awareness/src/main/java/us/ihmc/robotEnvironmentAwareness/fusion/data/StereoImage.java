package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.util.ArrayList;
import java.util.List;

public class StereoImage
{
   private final int numberOfPoints;
   private final int imageHeight;
   private final int imageWidth;
   private final StereoPoint[] stereoPoints;

   public StereoImage(int numberOfPoints, int imageHeight, int imageWidth)
   {
      this.numberOfPoints = numberOfPoints;
      this.imageHeight = imageHeight;
      this.imageWidth = imageWidth;

      stereoPoints = new StereoPoint[numberOfPoints];
   }

   public void setPoint(int pointIndex, StereoPoint stereoPoint)
   {
      this.stereoPoints[pointIndex] = stereoPoint;
   }

   public StereoPoint getPoint(int pointIndex)
   {
      return stereoPoints[pointIndex];
   }

   public StereoPoint[] getPoints()
   {
      return stereoPoints;
   }

   public int getHeight()
   {
      return imageHeight;
   }

   public int getWidth()
   {
      return imageWidth;
   }
}
