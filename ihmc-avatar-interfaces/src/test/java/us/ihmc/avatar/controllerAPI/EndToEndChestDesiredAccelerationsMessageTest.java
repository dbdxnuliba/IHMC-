package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.SpineDesiredAccelerationsMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.HandUserControlModeState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class EndToEndChestDesiredAccelerationsMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 18.1)
   @Test(timeout = 90000)
   public void testSimpleCommands() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();
      OneDoFJointBasics[] spineJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, chest);
      double[] chestDesiredJointAccelerations = RandomNumbers.nextDoubleArray(random, spineJoints.length, 0.1);
      SpineDesiredAccelerationsMessage desiredAccelerationsMessage = HumanoidMessageTools.createSpineDesiredAccelerationsMessage(chestDesiredJointAccelerations);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBodyControlMode defaultControlState = getRobotModel().getWalkingControllerParameters().getDefaultControlModesForRigidBodies().get(chest.getName());
      if (defaultControlState == null)
         defaultControlState = RigidBodyControlMode.JOINTSPACE;

      assertEquals(defaultControlState, findControllerState(scs));

      drcSimulationTestHelper.publishToController(desiredAccelerationsMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(HandUserControlModeState.TIME_WITH_NO_MESSAGE_BEFORE_ABORT - 0.05);
      assertTrue(success);

      assertEquals(RigidBodyControlMode.USER, findControllerState(scs));
      double[] controllerDesiredJointAccelerations = findControllerDesiredJointAccelerations(spineJoints, scs);
      assertArrayEquals(chestDesiredJointAccelerations, controllerDesiredJointAccelerations, 1.0e-10);
      double[] qpOutputJointAccelerations = findQPOutputJointAccelerations(spineJoints, scs);
      assertArrayEquals(chestDesiredJointAccelerations, qpOutputJointAccelerations, 1.0e-3);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.07);
      assertTrue(success);

      assertEquals(defaultControlState, findControllerState(scs));
   }

   @SuppressWarnings("unchecked")
   public RigidBodyControlMode findControllerState(SimulationConstructionSet scs)
   {
      String namespace = "utorsoManager";
      String state = namespace + "State";
      return ((YoEnum<RigidBodyControlMode>) scs.getVariable(namespace, state)).getEnumValue();
   }

   public double[] findQPOutputJointAccelerations(OneDoFJointBasics[] joints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         qdd_ds[i] = scs.getVariable(WholeBodyInverseDynamicsSolver.class.getSimpleName(), "qdd_qp_" + joints[i].getName()).getValueAsDouble();
      }
      return qdd_ds;
   }

   public double[] findControllerDesiredJointAccelerations(OneDoFJointBasics[] joints, SimulationConstructionSet scs)
   {
      double[] qdd_ds = new double[joints.length];
      String prefix = "utorsoUserMode";
      for (int i = 0; i < joints.length; i++)
      {
         String name = prefix + "_" + joints[i].getName() + "_qdd_d";
         qdd_ds[i] = scs.getVariable(name).getValueAsDouble();
      }
      return qdd_ds;
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
