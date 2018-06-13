package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.FootController;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class DoubleSupportMotionState extends MotionControllerState
{
   private static final MotionControllerStateEnum stateEnum = MotionControllerStateEnum.DOUBLE_SUPPORT;

   private final YoEnum<MotionControllerStateEnum> requestedState;
   private final RigidBodyControlManager chestManger;
   private final RigidBodyControlManager headManager;
   private final SideDependentList<RigidBodyControlManager> handManagers;
   private final SideDependentList<FootController> feetControllers;

   public DoubleSupportMotionState(YoEnum<MotionControllerStateEnum> requestedState, RigidBodyControlManager chestManager, RigidBodyControlManager headManager,
                                   SideDependentList<RigidBodyControlManager> handManagers, SideDependentList<FootController> feetControllers,
                                   YoVariableRegistry registry)
   {
      super(stateEnum);
      this.handManagers = handManagers;
      this.headManager = headManager;
      this.chestManger = chestManager;
      this.feetControllers = feetControllers;
      this.requestedState = requestedState;
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public void doAction()
   {
      if(requestedState.getEnumValue() != null)
      {
         
      }
      runControlManagers();
   }

   private void runControlManagers()
   {
      headManager.compute();
      chestManger.compute();
      for (RobotSide side : RobotSide.values)
      {
         handManagers.get(side).compute();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      
   }

   @Override
   public void doTransitionIntoAction()
   {
      headManager.holdInJointspace();
      chestManger.holdInJointspace();
      for (RobotSide side : RobotSide.values)
      {
         handManagers.get(side).holdInTaskspace();
      }
   }
}
