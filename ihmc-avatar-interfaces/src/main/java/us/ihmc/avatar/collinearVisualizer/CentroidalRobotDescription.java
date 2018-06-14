package us.ihmc.avatar.collinearVisualizer;

import java.awt.Color;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class CentroidalRobotDescription extends RobotDescription
{
   public CentroidalRobotDescription(double mass, double Ixx, double Ixy, double Ixz, double Iyy, double Iyz, double Izz)
   {
      this(mass, new DenseMatrix64F(3, 3, true, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz));
   }

   public CentroidalRobotDescription(double mass, double Ixx, double Iyy, double Izz)
   {
      this(mass, Ixx, 0.0, 0.0, Iyy, 0.0, Izz);
   }

   public CentroidalRobotDescription(CentroidalRobotPhysicalProperties physicalProperties)
   {
      this(physicalProperties.getMass(), physicalProperties.getInertia());
   }

   public CentroidalRobotDescription(double mass, DenseMatrix64F inertiaTensor)
   {
      super("CentroidalBot");
      FloatingJointDescription rootJoint = new FloatingJointDescription("RootJoint");
      LinkDescription rootLink = new LinkDescription("RootLink");
      LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
      rootLinkGraphics.addEllipsoid(0.025, 0.025, 0.025, new YoAppearanceRGBColor(Color.BLUE, 0.5));
      rootLink.setLinkGraphics(rootLinkGraphics);
      rootLink.setMass(mass);
      rootLink.setMomentOfInertia(inertiaTensor);
      rootJoint.setLink(rootLink);
      addRootJoint(rootJoint);
   }
}
