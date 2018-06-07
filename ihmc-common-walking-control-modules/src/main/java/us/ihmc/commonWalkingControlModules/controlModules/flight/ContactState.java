package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.Map;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Stores the contact state of a robot in 2.5D representation
 * @author Apoorv
 *
 */
public class ContactState
{
   /**
    * The duration of this contact state
    */
   public double duration;
   /**
    * The support polygons during the this contact state. Will be null / empty polygons in case foot is not in contact
    */
   public final SideDependentList<ConvexPolygon2D> footSupportPolygons = new SideDependentList<>();;
   /**
    * The pose of the foot in this contact state. Will be null / empty in case foot is not in contact
    */
   public final SideDependentList<FramePose3D> footPoses = new SideDependentList<>();
   /**
    * The state of each of the robot feet
    */
   public final SideDependentList<Boolean> footInContact = new SideDependentList<>();

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      for (RobotSide side : RobotSide.values)
      {
         footSupportPolygons.put(side, new ConvexPolygon2D());
         footPoses.put(side, new FramePose3D());
      }
      reset();
   }

   public void reset()
   {
      duration = Double.NaN;
      for (RobotSide side : RobotSide.values)
      {
         footSupportPolygons.get(side).clearAndUpdate();
         footPoses.get(side).setToNaN();
         footInContact.put(side, false);
      }
   }

   public void setSupportPolygons(ConvexPolygon2D leftSupportPolygonToSet, ConvexPolygon2D rightSupportPolygonToSet)
   {
      footSupportPolygons.get(RobotSide.LEFT).set(leftSupportPolygonToSet);
      footSupportPolygons.get(RobotSide.RIGHT).set(rightSupportPolygonToSet);
   }

   /**
    * Sets the support polygon for the specified foot. The support polygon is defined with respect to its pose {@link #getPose(RobotSide)}
    * @param supportPolygonToSet should be defined wrt to the specified pose 
    */
   public void setSupportPolygon(RobotSide side, ConvexPolygon2D supportPolygonToSet)
   {
      footSupportPolygons.get(side).setAndUpdate(supportPolygonToSet);
   }

   /**
    * Sets the support polygon specified. The polygon is transformed to {@link #getPose(RobotSide)}
    * @param supportPolygonToSet support polygon to set. Will be stored wrt to the specified pose
    */
   public void setSupportPolygon(RobotSide side, FrameConvexPolygon2d supportPolygonToSet)
   {
      ConvexPolygon2D footPolygon = footSupportPolygons.get(side);
      FramePose3D footPose = footPoses.get(side);
      footPolygon.set(supportPolygonToSet.getGeometryObject());
      TransformHelperTools.transformFromReferenceFrameToReferenceFrame(supportPolygonToSet.getReferenceFrame(), footPose.getReferenceFrame(), footPolygon);
      TransformHelperTools.transformFromReferenceFrameToPoseByProjection(footPose, footPolygon);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   /**
    * Returns the support polygon wrt to the specified {@code ReferenceFrame}
    * @param referenceFrame
    * @param supportPolygonToSet
    */
   public void getSupportPolygon(RobotSide side, ReferenceFrame referenceFrame, FrameConvexPolygon2d supportPolygonToSet)
   {
      FramePose3D footPose = footPoses.get(side);
      supportPolygonToSet.setIncludingFrame(footPose.getReferenceFrame(), footSupportPolygons.get(side));
      TransformHelperTools.transformFromPoseToReferenceFrameByProjection(footPose, supportPolygonToSet.getGeometryObject());
      supportPolygonToSet.changeFrame(referenceFrame);
   }

   /**
    * Returns the support polygon wrt to {@link #getPose(RobotSide)}
    * @param supportPolygonToSet
    */
   public void getSupportPolygon(RobotSide side, ConvexPolygon2D supportPolygonToSet)
   {
      supportPolygonToSet.set(footSupportPolygons.get(side));
   }

   public boolean isSupported()
   {
      return footInContact.get(RobotSide.LEFT) || footInContact.get(RobotSide.RIGHT);
   }

   public double getDuration()
   {
      return duration;
   }

   public void setFootInContact(boolean isLeftFootInContact, boolean isRightFootInContact)
   {
      footInContact.put(RobotSide.LEFT, isLeftFootInContact);
      footInContact.put(RobotSide.RIGHT, isRightFootInContact);
   }

   public void setFootInContact(RobotSide side, boolean isInContact)
   {
      footInContact.put(side, isInContact);
   }

   public boolean isFootInContact(RobotSide side)
   {
      return footInContact.get(side);
   }

   public void setFootPoses(FramePose3DReadOnly leftFootPose, FramePose3DReadOnly rightFootPose)
   {
      footPoses.get(RobotSide.LEFT).setIncludingFrame(leftFootPose);
      footPoses.get(RobotSide.RIGHT).setIncludingFrame(rightFootPose);
   }
   
   public void setFootPose(RobotSide side, FramePose3DReadOnly poseToSet)
   {
      footPoses.get(side).setIncludingFrame(poseToSet);
   }

   public void setFootPose(RobotSide side, FramePose2DReadOnly poseToSet)
   {
      footPoses.get(side).setIncludingFrame(poseToSet);
   }

   public FramePose3DReadOnly getPose(RobotSide side)
   {
      return footPoses.get(side);
   }

   public void getPosition(RobotSide side, FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(footPoses.get(side).getPosition());
   }

   public void setPosition(RobotSide side, FramePoint3DReadOnly positionToSet)
   {
      footPoses.get(side).setPosition(positionToSet);
   }

   public void getOrientation(RobotSide side, FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(footPoses.get(side).getOrientation());
   }

   public void setOrientation(RobotSide side, FrameQuaternionReadOnly orientationToSet)
   {
      footPoses.get(side).setOrientation(orientationToSet);
   }

   public void getSupportPolygonCentroid(RobotSide side, FramePoint3D framePointToSet)
   {
      FramePose3D footPose = footPoses.get(side);
      framePointToSet.setIncludingFrame(footPose.getReferenceFrame(), footSupportPolygons.get(side).getCentroid(), 0.0);
      TransformHelperTools.transformFromPoseToReferenceFrame(footPose, framePointToSet);
   }

   public void set(ContactState other)
   {
      this.duration = other.duration;
      for (RobotSide side : RobotSide.values)
      {
         footInContact.put(side, other.isFootInContact(side));
         footPoses.get(side).setIncludingFrame(other.getPose(side));
         other.getSupportPolygon(side, footSupportPolygons.get(side));
      }
   }

   public int getNumberOfSupportPolygonVertices(RobotSide side)
   {
      return footSupportPolygons.get(side).getNumberOfVertices();
   }

   public String toString()
   {
      String toString = "isSupported: " + isSupported() + ",\nDuration: " + duration;
      for (RobotSide side : RobotSide.values)
         toString += side.getCamelCaseNameForMiddleOfExpression() + ": Pose: " + footPoses.get(side).toString() + ", Support Polygon: "
               + footSupportPolygons.get(side).toString() + "\n";
      return toString;
   }
}