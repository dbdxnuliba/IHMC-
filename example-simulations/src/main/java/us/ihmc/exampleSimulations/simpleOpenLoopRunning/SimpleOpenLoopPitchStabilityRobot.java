package us.ihmc.exampleSimulations.simpleOpenLoopRunning;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class SimpleOpenLoopPitchStabilityRobot extends Robot
{
   private double bodyMass = 1;
   private double bodyRoGX = 0.4;
   private double bodyRoGY = 0.05;
   private double bodyRoGZ = 0.05;

   private final FloatingPlanarJoint bodyJoint;
   private final ExternalForcePoint groundContactPoint;

   public SimpleOpenLoopPitchStabilityRobot()
   {
      super("SimpleOpenLoopPitchStabilityRobot");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      bodyJoint = new FloatingPlanarJoint("body", this);

      Link bodyLink = new Link("body");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRoGX, bodyRoGY, bodyRoGZ);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addEllipsoid(bodyRoGX, bodyRoGY, bodyRoGZ, YoAppearance.Red());
      bodyLink.setLinkGraphics(linkGraphics);

      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      groundContactPoint = new GroundContactPoint("ef", this);
      bodyJoint.addExternalForcePoint(groundContactPoint);

      double forceVectorScale = 0.1;

      YoGraphicVector groundReactionForceVector = new YoGraphicVector("GroundForce", groundContactPoint.getYoPosition(), groundContactPoint.getYoForce(),
                                                                      forceVectorScale, YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("GroundReactionForce", groundReactionForceVector);

      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);
   }

   public void setGroundContactPositionXWithRespectToBody(double groundContactX)
   {
      double bodyX = bodyJoint.getQ_t1().getDoubleValue();
      Tuple3DBasics newOffset = new Vector3D(bodyX + groundContactX, 0.0, 0.0);
      groundContactPoint.setOffsetWorld(newOffset);
   }

   public void setGroundContactForceZ(double groundContactForceZ)
   {
      groundContactPoint.setForce(0.0, 0.0, groundContactForceZ);
   }

   public double getMass()
   {
      return bodyJoint.getLink().getMass();
   }

   public double getX()
   {
      return bodyJoint.getQ_t1().getDoubleValue();
   }

   public double getXVelocity()
   {
      return bodyJoint.getQd_t1().getDoubleValue();
   }

   public double getZ()
   {
      return bodyJoint.getQ_t2().getDoubleValue();
   }

   public double getZVelocity()
   {
      return bodyJoint.getQd_t2().getDoubleValue();
   }

   public double getPitch()
   {
      return bodyJoint.getQ_rot().getDoubleValue();
   }

   public double getPitchRate()
   {
      return bodyJoint.getQd_rot().getDoubleValue();
   }

   private final Matrix3D momentOfInertiaToPack = new Matrix3D();
   
   public double getRadiusOfGyration()
   {
      bodyJoint.getLink().getMomentOfInertia(momentOfInertiaToPack);
      return Math.sqrt(momentOfInertiaToPack.getElement(1, 1) / bodyMass);
   }

}