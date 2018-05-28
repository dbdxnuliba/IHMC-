package us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot;

import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class CentroidalRobotController
{
   public abstract void initialize();
   public abstract void doControl();
   public abstract void setOutputVariables(YoFramePoint desiredCoP, YoFrameVector desiredExternalForce);
   public abstract YoVariableRegistry getRegistry();
   public abstract void setCurrentState(CentroidalStateReadOnly state);
}
