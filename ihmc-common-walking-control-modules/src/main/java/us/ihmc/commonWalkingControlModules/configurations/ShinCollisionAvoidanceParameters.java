package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ShinCollisionAvoidanceParameters extends CollisionAvoidanceParameters
{
   /**
    * {@inheritDoc}
    */
   @Override
   public Vector3DReadOnly getFirstFrameOffset()
   {
      return new Vector3D(0.11, 0.0, 0.0);
   }

   /**
    * {@inheritDoc}
    * This is useful when stepping over some edges. The collision avoidance may be triggered when placing the foot over an obstacle.
    */
   @Override
   public boolean ignoreEdgesAtLowerHeight()
   {
      return true;
   }
}
