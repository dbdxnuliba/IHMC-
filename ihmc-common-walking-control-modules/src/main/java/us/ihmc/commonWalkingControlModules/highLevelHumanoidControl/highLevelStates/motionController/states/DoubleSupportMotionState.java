package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.footControl.FootController;
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
      this.handManagers = handManagers;
      this.headManager = headManager;
      this.chestManger = chestManager;
      this.feetControllers = feetControllers;
      this.requestedState = requestedState;
   }

   public boolean isDone()
   {
      return false;
   }

   @Override
   public void doAction(double timeInState)
   {
      if (requestedState.getEnumValue() != null)
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
         feetControllers.get(side).doControl();
      }
   }

   @Override
   public void onExit()
   {
      
   }

   @Override
   public void onEntry()
   {
      headManager.holdInJointspace();
      chestManger.holdInJointspace();
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyControlManager handManager = handManagers.get(side);
         handManager.holdInJointspace();
         FootController footController = feetControllers.get(side);
         footController.requestTransitionToContact(true, true);
      }
   }
}
