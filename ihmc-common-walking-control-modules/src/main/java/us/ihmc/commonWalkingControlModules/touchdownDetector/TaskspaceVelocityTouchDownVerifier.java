package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 *
 */
public class TaskspaceVelocityTouchDownVerifier implements TouchdownDetector
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final YoBoolean touchdownDetected;
   private final DoubleProvider soleVelocityThreshold;
   private final Twist soleTwist = new Twist();
   private final YoFrameVector3D soleVelocity;
   private final MovingReferenceFrame soleFrame;

   public TaskspaceVelocityTouchDownVerifier(MovingReferenceFrame soleFrame, RobotQuadrant robotQuadrant, DoubleProvider soleVelocityThreshold, YoVariableRegistry parentRegistry)
   {
      String prefix = robotQuadrant.getCamelCaseName();
      String fullName = prefix + name;
      this.registry = new YoVariableRegistry(fullName);
      this.soleFrame = soleFrame;
      this.soleVelocityThreshold = soleVelocityThreshold;

      this.touchdownDetected = new YoBoolean(prefix + "SoleVelocityTouchdownDetected", registry);
      this.soleVelocity = new YoFrameVector3D(prefix + "SoleVelocity", ReferenceFrame.getWorldFrame(), registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasTouchedDown()
   {
      return touchdownDetected.getBooleanValue();
   }

   @Override
   public void update()
   {
      soleFrame.getTwistOfFrame(soleTwist);
      soleTwist.changeFrame(ReferenceFrame.getWorldFrame());
      soleVelocity.set(soleTwist.getLinearPart());
      touchdownDetected.set(soleVelocity.length() < soleVelocityThreshold.getValue());
   }

   @Override
   public void reset()
   {
      touchdownDetected.set(false);
   }

   @Override
   public String getName()
   {
      return name;
   }
}
