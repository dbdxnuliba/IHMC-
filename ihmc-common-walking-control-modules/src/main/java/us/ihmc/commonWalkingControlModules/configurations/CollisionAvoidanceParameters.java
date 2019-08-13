package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Parameters for a generic collision avoidance manager. The manager works by approximating the body of interest
 * with a line connecting two reference frames. These two frames are supposed to be rigidly attached to the body
 * of interest. When active, it computes the minimum distance from this line to any of the planar regions sent with
 * the corresponding command. It also compute the distance direction. In order to avoid collisions, a reference point 
 * is set along such direction passing through the closest point on the body (the point which is probably
 * going to collide), at a distance equal to the activation threshold. A PD is used to drive the closest point on
 * the body to the reference point defined above. Only the error along the distance direction is weighted, such that 
 * the motion of the body is not influenced in all the other directions. 
 * 
 * @author Stefano Dafarra
 *
 */
public class CollisionAvoidanceParameters
{
   /**
    * Determine whether to use the collision avoidance manager or not. If false, the manager is not created.
    */
   public boolean useCollisionAvoidance()
   {
      return false;
   }

   /**
    * Distance below which the manager is activated (assuming that {@link useCollisionAvoidance} is true).
    */
   public double getActivationThreshold()
   {
      return 0.1;
   }

   /**
    * Distance above which the manager is deactivated. I.e., even if {@link useCollisionAvoidance} is true, if the
    * minimum distance of the body of interest is greater than this value for every planar region, the manager has
    * no effect. This value has to be greater than the activation threshold({@link getActivationThreshold}).
    */
   public double getDeactivationThreshold()
   {
      return 1.2 * getActivationThreshold();
   }

   /**
    * Offset to be added to the position of the first frame. This offset is defined in the first frame coordinates.
    */
   public Vector3DReadOnly getFirstFrameOffset()
   {
      return new Vector3D(0.0, 0.0, 0.0);
   }

   /**
    * Offset to be added to the position of the second frame. This offset is defined in the second frame coordinates.
    */
   public Vector3DReadOnly getSecondFrameOffset()
   {
      return new Vector3D(0.0, 0.0, 0.0);
   }

   /**
    * When computing the minimum distance direction, it is possible to set an upper-bound to the vertical component.
    * This allow to constrain the corresponding motion to have always an upward component (the motion is in the direction 
    * opposite to the minimum distance direction). Set to a value greater than +1.0 to disable such saturation
    */
   public double getMaximumVerticalDistanceComponent()
   {
      return -0.1;
   }

   /**
    * If the closest point on the obstacle has a z-component lower than that of both end points, the collision avoidance is disabled.
    */
   public boolean ignoreEdgesAtLowerHeight()
   {
      return false;
   }

   /**
    * The weight of the collision avoidance task in the walking controller cost function.
    */
   public double getWeight()
   {
      return 10.0;
   }

   /**
    * Proportional gain used in the PD controller employed to drive the body toward the reference point away from the
    * collision. 
    */
   public double getProportionalGain()
   {
      return 500.0;
   }

   /**
    * Derivative gain used in the PD controller employed to drive the body toward the reference point away from the
    * collision. 
    */
   public double getDerivativeGain()
   {
      return 5.0 * Math.sqrt(getProportionalGain());
   }

   /**
    * Max feedback the PD controller employed to drive the body toward the reference point away from the
    * collision. 
    */
   public double getMaxFeedback()
   {
      return 250.0;
   }

   /**
    * Max feedback variation the PD controller employed to drive the body toward the reference point away from the
    * collision. 
    */
   public double getMaxFeedbackVariation()
   {
      return 10.0 * getMaxFeedback();
   }

}
