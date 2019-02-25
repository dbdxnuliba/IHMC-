package us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.partNames.LegJointName;

public class VaryingHeightAngleAndDistanceEvaluator
{

   public boolean getDistancePosAlignment(FramePoint3D com3D, FramePoint2D projectedCMP, FrameVector2D icpError)
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(projectedCMP);
      boolean distancePosAlignment = (MathTools.sign(copCoMVec.getY()) == MathTools.sign(icpError.getY()));
      return distancePosAlignment;
   }

   public boolean getDistanceImproves(FrameVector2D centerOfMassVelocity2D, FrameVector2D icpError)
   {
      double distanceImprovingAngle = icpError.angle(centerOfMassVelocity2D);
      boolean distanceImproves;
      if (distanceImprovingAngle > -0.5 * Math.PI && distanceImprovingAngle < 0.5 * Math.PI)
      {
         distanceImproves = true;
      }
      else
      {
         distanceImproves = false;
      }
      return distanceImproves;
   }

   public boolean getAngleGrows(double errorAngle, FrameVector2D centerOfMassVelocity2D, FramePoint2D projectedDesiredCMP, FramePoint3D com3D)
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(projectedDesiredCMP);
      double comDirectionAngle = centerOfMassVelocity2D.angle(copCoMVec);
      boolean angleGrows = errorAngle / comDirectionAngle > 0;
      if (errorAngle < 0)
      {
         angleGrows = !angleGrows;
      }
      return angleGrows;
   }

   // calculates the error angle
   public double getErrorAngle(FrameVector2D icpError, FramePoint2D projectedDesiredCMP, FramePoint3D com3D)
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(projectedDesiredCMP);
      double errorAngle = copCoMVec.angle(icpError);
      return errorAngle;
   }

   // Currently NOT USED, meant to see if angle comes closer to pos align thresh
   public boolean getAngleImproves(boolean angleGrows, double errorAngle, double posAlignTresh, double negAlignTresh)
   {
      boolean angleImproves;
      if (angleGrows == true && (errorAngle > -posAlignTresh && errorAngle < 0 || errorAngle > negAlignTresh))
      {
         angleImproves = true;
      }
      else
      {
         angleImproves = false;
      }
      return angleImproves;
   }

   // error angle end of swing
   public double getErrorAngleEndOfSwing(FramePoint2DReadOnly com2DEndOfSwing, FramePoint2D projectedDesiredCMP, FrameVector2D icpError)
   {
      double errorAngleEndOfSwing;
      FrameVector2D copCoMEndOfSwingVec = new FrameVector2D();
      copCoMEndOfSwingVec.setIncludingFrame(com2DEndOfSwing);
      copCoMEndOfSwingVec.changeFrame(ReferenceFrame.getWorldFrame());
      copCoMEndOfSwingVec.sub(projectedDesiredCMP);
      errorAngleEndOfSwing = copCoMEndOfSwingVec.angle(icpError);
      return errorAngleEndOfSwing;
   }

   // Determines which strategy to use: angle or distance.
   public boolean getUseAngleForConditions(FrameVector2D icpError)
   {
      boolean useAngleForConditions;
      FrameVector2D walkingDirection = new FrameVector2D();
      walkingDirection.changeFrame(ReferenceFrame.getWorldFrame());
      walkingDirection.set(1.0, 0.0);
      double angle = walkingDirection.angle(icpError);

      if (Math.abs(angle) < 0.20)
      {
         useAngleForConditions = true;
      }
      else if (Math.abs(angle) > Math.PI - 0.7)
      {
         useAngleForConditions = true;
      }
      else
      {
         useAngleForConditions = false;
      }
      return useAngleForConditions;
   }
}
