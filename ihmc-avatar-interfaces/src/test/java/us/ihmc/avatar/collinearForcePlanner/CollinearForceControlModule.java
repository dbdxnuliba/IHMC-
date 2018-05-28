package us.ihmc.avatar.collinearForcePlanner;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalRobotController;
import us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot.CentroidalStateReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CollinearForceControlModule extends CentroidalRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final String namePrefix = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(namePrefix + "Registry");
   private final YoFrameVector gravity;
   private final YoDouble mass;
   
   private CentroidalStateReadOnly state;
   private YoFramePoint desiredCoP;
   private YoFrameVector desiredGroundReactionForce;
   
   // Variables for calculations
   private final FrameVector3D tempVector = new FrameVector3D(); 

   public CollinearForceControlModule()
   {
      this.mass = new YoDouble(namePrefix + "Mass", registry);
      this.gravity = new YoFrameVector(namePrefix + "Gravity", worldFrame, registry);
   }

   public void setupController(double robotMass, FrameVector3DReadOnly gravity)
   {
      this.mass.set(robotMass);
      tempVector.setIncludingFrame(gravity);
      tempVector.changeFrame(worldFrame);
      this.gravity.set(tempVector);
   }
   
   @Override
   public void initialize()
   {
      if(state == null)
         throw new RuntimeException("Estimated state has not been provided to the controller");
      intializeForStanding();
   }

   private void intializeForStanding()
   {
      desiredGroundReactionForce.set(gravity);
      desiredGroundReactionForce.scale(-mass.getDoubleValue());
   }

   @Override
   public void doControl()
   {
      // TODO implemenent this
   }

   @Override
   public void setOutputVariables(YoFramePoint desiredCoPToSet, YoFrameVector desiredExternalForce)
   {
      desiredCoP = desiredCoPToSet;
      desiredGroundReactionForce = desiredExternalForce;
   }

   @Override
   public void setCurrentState(CentroidalStateReadOnly stateToSet)
   {
      state = stateToSet;
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
