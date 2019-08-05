package us.ihmc.atlas.collisionAvoidance;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.collisionAvoidance.AvatarCollisionAvoidanceTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.ShinCollisionAvoidanceParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasCollisionAvoidanceTest extends AvatarCollisionAvoidanceTest
{
   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModelForCollisionAvoidanceTest(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);

   @Test
   public void testWalkingUpOfMediumStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.35);
      super.walkUpToHighStep(stepHeights);
   }

   @Test
   public void testWalkingUpOfMediumStepWithCustomTrajectories() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.40);
      super.walkUpToHighStepWithCustomTrajectories(stepHeights, false);
   }

   @Test
   public void testWalkingUpOfHighStepWithCustomTrajectories() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.45);
      super.walkUpToHighStepWithCustomTrajectories(stepHeights, false);
   }

   @Test
   public void testWalkingUpAndDownOf2MediumStepsAndASmallStep() throws SimulationExceededMaximumTimeException
   {
      ArrayList<Double> stepHeights = new ArrayList<Double>();
      stepHeights.add(0.3);
      stepHeights.add(0.6);
      stepHeights.add(0.3);
      stepHeights.add(0.5);
      stepHeights.add(0.8);
      stepHeights.add(0.6);

      super.walkUpToHighStep(stepHeights);
   }


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
   
   public class AtlasRobotModelForCollisionAvoidanceTest extends AtlasRobotModel
   {
      public class EnabledShinCollisionAvoidanceParameters extends ShinCollisionAvoidanceParameters
      {
         @Override
         public boolean useCollisionAvoidance()
         {
            return true;
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
