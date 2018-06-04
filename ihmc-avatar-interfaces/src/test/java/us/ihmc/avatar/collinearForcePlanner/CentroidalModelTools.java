package us.ihmc.avatar.collinearForcePlanner;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * A set of functions to help move between momentum rate of change and CoP / ground reaction force representations
 * <p> Does not use the Linear Inverted Pendulum model to allow for validity of results in a larger number of cases </p>
 * @author Apoorv S
 *
 */
public class CentroidalModelTools
{
   public static void computeCenterOfPressureForFlatGround(FramePoint3DReadOnly centerOfMass, double groundHeight, FrameVector3DReadOnly groundReactionForce,
                                                           FrameVector3DReadOnly angularMomentumRateOfChange, FramePoint3D centerOfPressureToSet)
   {
      centerOfMass.checkReferenceFrameMatch(groundReactionForce);
      centerOfMass.checkReferenceFrameMatch(angularMomentumRateOfChange);
      double Fx = groundReactionForce.getX();
      double Fy = groundReactionForce.getY();
      double Fz = groundReactionForce.getZ();
      if(Math.abs(Fz)< Epsilons.ONE_HUNDRED_THOUSANDTH)
      {
         centerOfPressureToSet.setIncludingFrame(centerOfMass);
         centerOfPressureToSet.subZ(groundHeight);
         return;
      }
      double Mx = angularMomentumRateOfChange.getX();
      double My = angularMomentumRateOfChange.getY();
      double Mz = angularMomentumRateOfChange.getZ();
      double forceMagnitudeSquaredInverse = 1.0 / (Fx * Fx + Fy * Fy + Fz * Fz);
      double rx = (Fy * Mz - Fz * My) * forceMagnitudeSquaredInverse;
      double ry = (Fz * Mx - Fx * Mz) * forceMagnitudeSquaredInverse;
      double rz = (Fx * My - Fy * Mx) * forceMagnitudeSquaredInverse;
      double x = centerOfMass.getX();
      double y = centerOfMass.getY();
      double z = centerOfMass.getZ();
      double scalar = (rz + z - groundHeight) / Fz;
      double copX = x + rx - scalar * Fx;
      double copY = y + ry - scalar * Fy;
      centerOfPressureToSet.setIncludingFrame(centerOfMass.getReferenceFrame(), copX, copY, groundHeight);
   }

   public static void computeGroundReactionForce(FrameVector3DReadOnly linearMomentumRateOfChange, FrameVector3DReadOnly gravity, double mass,
                                                 FrameVector3D groundReactionForceToSet)
   {
      linearMomentumRateOfChange.checkReferenceFrameMatch(gravity);
      groundReactionForceToSet.set(gravity);
      groundReactionForceToSet.scale(-mass);
      groundReactionForceToSet.add(linearMomentumRateOfChange);
   }
}
