package us.ihmc.atlas.stepUpPlanner;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.stepUpPlanner.AvatarStepUpPlannerTest;
import us.ihmc.commonWalkingControlModules.configurations.ShinCollisionAvoidanceParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasStepUpPlannerTest extends AvatarStepUpPlannerTest
{

   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModelForCollisionAvoidanceTest(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   @Test
   public void testStepUpPlannerLowStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.30);
   }

   @Test
   public void testStepUpPlannerMediumStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.35);
   }

   @Test
   public void testStepUpPlannerHighStep() throws SimulationExceededMaximumTimeException
   {
      super.walkUpToHighStep(0.40);
   }

   public class AtlasRobotModelForCollisionAvoidanceTest extends AtlasRobotModel
   {
      public class EnabledShinCollisionAvoidanceParameters extends ShinCollisionAvoidanceParameters
      {
         @Override
         public boolean useCollisionAvoidance()
         {
            return true;
         }

         @Override
         public double getWeight()
         {
            return 2.5;
         }

         @Override
         public double getProportionalGain()
         {
            return 300.0;
         }
      }

      public class EnabledShinCollisionsWalkingControllerParameters extends AtlasWalkingControllerParameters
      {
         private final SideDependentList<ShinCollisionAvoidanceParameters> shinsCollisionAvoidanceParameters;

         public EnabledShinCollisionsWalkingControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
         {
            super(target, jointMap, contactPointParameters);
            shinsCollisionAvoidanceParameters = new SideDependentList<ShinCollisionAvoidanceParameters>(new EnabledShinCollisionAvoidanceParameters(),
                                                                                                        new EnabledShinCollisionAvoidanceParameters());
         }

         public SideDependentList<ShinCollisionAvoidanceParameters> getShinsCollisionAvoidanceParameters()
         {
            return shinsCollisionAvoidanceParameters;
         }

      }

      private final EnabledShinCollisionsWalkingControllerParameters walkingParameters;

      public AtlasRobotModelForCollisionAvoidanceTest(AtlasRobotVersion atlasVersion, RobotTarget target)
      {
         super(atlasVersion, target);
         walkingParameters = new EnabledShinCollisionsWalkingControllerParameters(target, super.getJointMap(), super.getContactPointParameters());
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return walkingParameters;
      }

   }

}
