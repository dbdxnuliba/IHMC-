package us.ihmc.manipulation.planning.planarRobot;

import us.ihmc.euclid.tuple2D.Point2D;

public class PlanarRobotJoint
{
   private final PlanarRobotJoint parentJoint;

   private final double linkLength;

   private double jointAngle;
   private double endTipAngle;

   private final Point2D jointPoint = new Point2D();
   private final Point2D endTipPoint = new Point2D();

   public PlanarRobotJoint(Point2D basePoint, double linkLength, double initialAngle)
   {
      this.parentJoint = null;
      this.linkLength = linkLength;
      setJointAngle(initialAngle);
      updateJoint();
   }

   public PlanarRobotJoint(PlanarRobotJoint parentJoint, double linkLength, double initialAngle)
   {
      this.parentJoint = parentJoint;
      this.linkLength = linkLength;
      setJointAngle(initialAngle);
      updateJoint();
   }

   public void setJointAngle(double jointAngle)
   {
      // to set joint angle.
      this.jointAngle = jointAngle;
   }

   public void updateJoint()
   {
      // to update end tip angle, joint point, end tip point.
      if (parentJoint == null)
      {
         this.endTipAngle = jointAngle;
         this.endTipPoint.set(jointPoint);
         this.endTipPoint.add(linkLength * Math.cos(endTipAngle), linkLength * Math.sin(endTipAngle));
      }
      else
      {
         this.jointPoint.set(parentJoint.endTipPoint);
         this.endTipAngle = parentJoint.endTipAngle + jointAngle;
         this.endTipPoint.set(parentJoint.endTipPoint);
         this.endTipPoint.add(linkLength * Math.cos(endTipAngle), linkLength * Math.sin(endTipAngle));
      }
   }
   
   public PlanarRobotJoint getParentJoint()
   {
      return parentJoint;
   }

   public Point2D getJointPoint()
   {
      return jointPoint;
   }

   public Point2D getEndTip()
   {
      return endTipPoint;
   }
   
   public double getEndTipAngle()
   {
      return endTipAngle;
   }
   
   public double getJointAngle()
   {
      return jointAngle;
   }
   
   public double getLinkLength()
   {
      return linkLength;
   }
}
