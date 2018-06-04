package us.ihmc.avatar.collinearForcePlanner;

import static org.junit.Assert.assertTrue;

import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CentroidalRobotControllerCoreTest
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test(timeout = 1000)
   public void testInitialization()
   {
      CentroidalRobotControllerCore controllerCore = createControllerCore();
      assertTrue(controllerCore != null);
   }

   private CentroidalRobotControllerCore createControllerCore()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(Stream.of(new Point2D(0.1, 0.1), new Point2D(-0.1, 0.1), new Point2D(0.1, -0.1),
                                                                         new Point2D(-0.1, -0.1))
                                                                     .collect(Collectors.toList()));
      SideDependentList<ConvexPolygon2D> footSupportPolygons = new SideDependentList<>(defaultFootPolygon, defaultFootPolygon);
      CentroidalRobotControllerCore controllerCore = new CentroidalRobotControllerCore(footSupportPolygons, registry, null);
      return controllerCore;
   }

   @Test(timeout = 1000)
   public void testStationary()
   {
      CentroidalRobotControllerCore controllerCore = createControllerCore();
      controllerCore.setCoefficientOfFriction(1.0);
      controllerCore.setAngularMomentumWeights(new FrameVector3D(worldFrame, 0.1, 0.1, 0.1));
      controllerCore.setLinearMomentumWeights(new FrameVector3D(worldFrame, 1.0, 1.0, 1.0));
      controllerCore.setDesiredLinearMomentumRateOfChange(new FrameVector3D(worldFrame, 0.0, 0.1, 1.0));
      controllerCore.setDesiredAngularMomentumRateOfChange(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      FramePoint3D centerOfMass = new FramePoint3D(worldFrame, 0.0, 0.0, 0.45);
      controllerCore.setCenterOfMassLocation(centerOfMass);
      controllerCore.setFeetLocations(new FramePoint3D(worldFrame, 0.1, 0.15, 0.0), new FramePoint3D(worldFrame, 0.0, -0.15, 0.0));
      controllerCore.updateFootContactState(true, true);
      controllerCore.compute();
      FrameVector3D achievedLinearMomentum = new FrameVector3D();
      FrameVector3D achievedAngularMomentum = new FrameVector3D();
      controllerCore.getAchievedLinearMomentumRateOfChange(achievedLinearMomentum);
      controllerCore.getAchievedLinearMomentumRateOfChange(achievedAngularMomentum);
      PrintTools.debug(achievedLinearMomentum.toString());
      PrintTools.debug(achievedAngularMomentum.toString());
      FrameVector3D groundReactionForce = new FrameVector3D();
      FramePoint3D centerOfPressure = new FramePoint3D();
      CentroidalModelTools.computeGroundReactionForce(achievedLinearMomentum, new FrameVector3D(worldFrame, 0.0, 0.0, -9.81), 1.0, groundReactionForce);
      CentroidalModelTools.computeCenterOfPressureForFlatGround(centerOfMass, 0.0, groundReactionForce, achievedAngularMomentum, centerOfPressure);
      PrintTools.debug("Ground reaction: " + groundReactionForce.toString());
      PrintTools.debug("CoP: " + centerOfPressure.toString());
   }
}