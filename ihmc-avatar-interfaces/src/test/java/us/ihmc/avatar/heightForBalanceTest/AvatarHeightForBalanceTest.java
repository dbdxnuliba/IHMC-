package us.ihmc.avatar.heightForBalanceTest;


import com.vividsolutions.jts.math.Vector2D;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.Test;
import org.opencv.core.Mat;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryTestSetup;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.yoVariables.variable.YoBoolean;
import java.io.FileWriter;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertTrue;

public abstract class AvatarHeightForBalanceTest extends AvatarHeightForBalanceTestSetup implements MultiRobotTestInterface
{
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushStanding() throws Exception
   {
      FootstepDataListMessage footsteps = createStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, false);


      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);


      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05;
      pushRobotController.applyForce(forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushFrontalStanding() throws Exception
   {
      FootstepDataListMessage footsteps = createStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, false);


      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);


      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05;
      pushRobotController.applyForce(forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushContraDiagonalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushContraDiagonalFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, -0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushDiagonalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, -0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushDiagonalFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.5, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushFrontalInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushBackInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushRightInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushLeftInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.0 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.05 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @Test()
   public void testIterativePush() throws Exception
   {
      FileWriter writer = new FileWriter("angleAndPercentWeight.txt");
      ArrayList<Vector2D> angleAndPercentWeight = new ArrayList<Vector2D>();
      for(int i=0; i<12; i++)
      {
         double angle = i*30*Math.PI/180;
         percentWeight = 0.30;
         for (int j = 0; j < 10000; j++)
         {
            percentWeight += 0.01;
            FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
            footsteps.setAreFootstepsAdjustable(false);
            setupAndRunTest(footsteps, true);
            drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
            // push timing:
            StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
            double delay = 0.0 * swingTime;

            // push parameters:
            Vector3D forceDirection = new Vector3D(Math.cos(angle), Math.sin(angle), 0.0);
            double magnitude = percentWeight * totalMass * 9.81;
            double duration = 0.05 * swingTime;
            pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
            List<FootstepDataMessage> footstepList = footsteps.getFootstepDataList();
            int size = footstepList.size();
            duration = size * (footsteps.getDefaultSwingDuration() + footsteps.getDefaultTransferDuration());
            boolean succes = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 1.0);
            drcSimulationTestHelper.destroySimulation();
            if(!succes)
            {
               Vector2D resultVector = new Vector2D(angle,percentWeight-0.01);
               angleAndPercentWeight.add(resultVector);
               writer.write(" " +  angle + " " + percentWeight + System.lineSeparator());
               continue;
            }
         }
      }
      writer.close();

   }
}
