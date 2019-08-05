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
}
