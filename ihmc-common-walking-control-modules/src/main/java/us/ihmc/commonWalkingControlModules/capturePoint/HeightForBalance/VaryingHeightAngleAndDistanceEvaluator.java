package us.ihmc.commonWalkingControlModules.capturePoint.HeightForBalance;

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

   public boolean getDistancePosAlignment(double errorAngle)
   {
      boolean distancePosAlignment;
      if (errorAngle < 0.5 * Math.PI && errorAngle > -0.5 * Math.PI)
      {
         distancePosAlignment = true;
      }
      else
      {
         distancePosAlignment = false;
      }
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

   public double getErrorAngle(FrameVector2D icpError, FramePoint2D projectedDesiredCMP, FramePoint3D com3D)
   {
      FrameVector2D copCoMVec = new FrameVector2D();
      copCoMVec.setIncludingFrame(com3D);
      copCoMVec.sub(projectedDesiredCMP);
      double errorAngle = copCoMVec.angle(icpError);
      return errorAngle;
   }

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

   public double getErrorAngleEndOfSwing(FramePoint2DReadOnly com2DEndOfSwing, FramePoint2D projectedDesiredCMP, FrameVector2D icpError )
   {
      double errorAngleEndOfSwing;
      FrameVector2D copCoMEndOfSwingVec = new FrameVector2D();
      copCoMEndOfSwingVec.setIncludingFrame(com2DEndOfSwing);
      copCoMEndOfSwingVec.changeFrame(ReferenceFrame.getWorldFrame());
      copCoMEndOfSwingVec.sub(projectedDesiredCMP);
      errorAngleEndOfSwing = copCoMEndOfSwingVec.angle(icpError);
      return errorAngleEndOfSwing;
   }

   public boolean getUseAngleForConditions(FrameConvexPolygon2D supportPolygon, FramePoint2D projectedDesiredCMP, FramePoint2DReadOnly comEndOfSwing2D)
   {
      boolean useAngleForConditions;
      LineSegment2D projectedCMPPolygonEdge = new LineSegment2D();
      boolean closestEdgeFound = supportPolygon.getClosestEdge(projectedDesiredCMP, projectedCMPPolygonEdge);
      FramePoint2D closestVertex = new FramePoint2D();
      supportPolygon.getClosestVertex(projectedDesiredCMP, closestVertex);

      // FRONT EDGE: Checks if the polygon edge the CoP is projected on is the 'front' one. Hacky, should be in walking direction and not in x.
      if (MathTools.epsilonEquals(projectedCMPPolygonEdge.getFirstEndpointX(), supportPolygon.getMaxX(), 0.04)
            && MathTools.epsilonEquals(projectedCMPPolygonEdge.getSecondEndpointX(), supportPolygon.getMaxX(),0.04))
      {
         useAngleForConditions = true;
      }

      // CLOSE TO FRONT EDGE: Checks if the projected CoP is close to the 'front' polygon edge. Also hacky.
      else if (MathTools.epsilonEquals(projectedDesiredCMP.getX(),supportPolygon.getMaxX(),0.02))
      {
         useAngleForConditions = true;
      }

      // BACK EDGE:
      else if (MathTools.epsilonEquals(projectedCMPPolygonEdge.getFirstEndpointX(), supportPolygon.getMinX(), 0.04)
            && MathTools.epsilonEquals(projectedCMPPolygonEdge.getSecondEndpointX(), supportPolygon.getMinX(),0.04) && closestVertex.distance(projectedDesiredCMP)>0.03)
      {
         useAngleForConditions = true;
      }

      // ELSE: (don't use angle)
      else
      {
         useAngleForConditions = false;
      }
      return  useAngleForConditions;
   }
}