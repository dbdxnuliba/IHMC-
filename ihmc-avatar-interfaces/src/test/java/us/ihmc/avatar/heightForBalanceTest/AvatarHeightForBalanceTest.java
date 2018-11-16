package us.ihmc.avatar.heightForBalanceTest;


import com.vividsolutions.jts.math.Vector2D;
import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.io.FileWriter;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

public abstract class AvatarHeightForBalanceTest extends AvatarHeightForBalanceTestSetup implements MultiRobotTestInterface
{
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushStanding() throws Exception
   {
      FootstepDataListMessage footsteps = createStandingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, false, false);


      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);


      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
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
      setupAndRunTest(footsteps, false, false);


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
   public void testPushAngle() throws Exception
   {
      //angle=angle*Math.PI/180;
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      footsteps.setAreFootstepsAdjustable(false);
      setupAndRunTest(footsteps, true, false);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(Math.cos(angle), Math.sin(angle), 0.0);
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
      setupAndRunTest(footsteps, true, false);
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
      setupAndRunTest(footsteps, true, false);
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
      setupAndRunTest(footsteps, true, false);
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
      setupAndRunTest(footsteps, true, false);
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
   public void testIterativePushStanding() throws Exception
   {
      FileWriter writer = new FileWriter("standingCase.txt");
      ArrayList<Vector2D> standingCase = new ArrayList<Vector2D>();
      double increment= 0.001;
         for (int j = 0; j < 10000; j++)
         {
            percentWeight += increment;
            LogTools.info("Current percentWeight = " + percentWeight);
            FootstepDataListMessage footsteps = createStandingFootstepMessage();
            footsteps.setAreFootstepsAdjustable(false);
            setupAndRunTest(footsteps, false, false);


            drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);


            // push parameters:
            Vector3D forceDirection = new Vector3D(1.0, 0, 0.0);
            double magnitude = percentWeight * totalMass * 9.81;
            double duration = 0.05;
            pushRobotController.applyForce(forceDirection, magnitude, duration);
            boolean succes;
            try
            {
               succes = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
            }
            catch (Exception e)
            {
               LogTools.info(e.getMessage());
               succes = false;
            }

            drcSimulationTestHelper.destroySimulation();
            if(!succes)
            {
               percentWeight=percentWeight-increment;
               writer.write(" " +  + percentWeight + System.lineSeparator());
               break;
            }
         }
      writer.close();
      Assert.assertTrue(true);
   }
}
