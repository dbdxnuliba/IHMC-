package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class CollisionAvoidanceParameters
{
   private boolean useCollisionAvoidance_ = false;

   public boolean useCollisionAvoidance()
   {
      return useCollisionAvoidance_;
   }

   public void setUseCollisionAvoidance(boolean useCollisionAvoidance)
   {
      useCollisionAvoidance_ = useCollisionAvoidance;
   }

   public double getActivationThreshold()
   {
      return 0.1;
   }

   public double getDeactivationThreshold()
   {
      return 1.2 * getActivationThreshold();
   }

   public Vector3DReadOnly getFirstFrameOffset()
   {
      return new Vector3D(0.0, 0.0, 0.0);
   }

   public Vector3DReadOnly getSecondFrameOffset()
   {
      return new Vector3D(0.0, 0.0, 0.0);
   }

   public double getMaximumVerticalDistanceComponent()
   {
      return -0.1;
   }

   public double getWeight()
   {
      return 10.0;
   }

   public double getProportionalGain()
   {
      return 500.0;
   }

   public double getDerivativeGain()
   {
      return 5.0 * Math.sqrt(getProportionalGain());
   }

   public double getMaxFeedback()
   {
      return 250.0;
   }

   public double getMaxFeedbackVariation()
   {
      return 10.0 * getMaxFeedback();
   }

}
