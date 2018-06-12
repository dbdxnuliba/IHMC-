package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.FootTrajectoryGenerator;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetTrajectoryGenerationTest
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test(timeout = 1000)
   public void testTrajectoryGeneration()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      FootTrajectoryGenerator trajectoryGenerator = new FootTrajectoryGenerator(RobotSide.LEFT, registry, null);
      trajectoryGenerator.initialize(0.4, 0.4, new FrameVector3D(worldFrame, 0.0, 0.0, -0.1), new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), 0.1);
      double duration = 0.6;
      trajectoryGenerator.setDuration(duration);
      FramePoint3D initialPosition = new FramePoint3D();
      FrameVector3D initialVelocity = new FrameVector3D();
      FrameVector3D initialAcceleration = new FrameVector3D();
      FramePoint3D finalPosition = new FramePoint3D();
      trajectoryGenerator.setInitialConditions(initialPosition, initialVelocity, initialAcceleration);
      trajectoryGenerator.setFinalConditions(finalPosition);
      trajectoryGenerator.updateTrajectory(0.0);
      double dt = 0.01;
      FramePoint3D trajectoryPosition = new FramePoint3D();
      FrameVector3D trajectoryVelocity = new FrameVector3D();
      FrameVector3D trajectoryAcceleration = new FrameVector3D();
      for (double t = 0.0; t <= duration + dt; t += dt)
      {
         trajectoryGenerator.compute(t, trajectoryPosition, trajectoryVelocity, trajectoryAcceleration);
         PrintTools.debug("Time: " + String.format("%.3f", t)+ ", Pos: " + trajectoryPosition.toString() + ", Vel: " + trajectoryVelocity.toString() + ", Acc: "
               + trajectoryAcceleration.toString());
      }
   }
}
