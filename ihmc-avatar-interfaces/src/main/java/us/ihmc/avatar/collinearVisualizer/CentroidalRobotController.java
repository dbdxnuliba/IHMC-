package us.ihmc.avatar.collinearVisualizer;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.CentroidalStateReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public abstract class CentroidalRobotController
{
   public abstract void initialize();
   public abstract void doControl();
   public abstract void setOutputVariables(YoFramePoint3D desiredCoP, YoFrameVector3D desiredExternalForce);
   public abstract YoVariableRegistry getRegistry();
   public abstract void setCurrentState(CentroidalStateReadOnly state);
}