package us.ihmc.footstepPlanning.graphSearch.collision;

public class BodyCollisionData
{
   /**
    * Whether bounding box collision was detected
    */
   private boolean collisionDetected = false;

   /**
    * Distance of closest detected point to bounding box
    */
   private double distanceFromBoundingBox = Double.POSITIVE_INFINITY;

   void setDistanceFromBoundingBox(double distanceFromBoundingBox)
   {
      this.distanceFromBoundingBox = distanceFromBoundingBox;
   }

   void setCollisionDetected()
   {
      this.collisionDetected = true;
      this.distanceFromBoundingBox = 0.0;
   }

   public double getDistanceFromBoundingBox()
   {
      return distanceFromBoundingBox;
   }

   public boolean isCollisionDetected()
   {
      return collisionDetected;
   }
}
