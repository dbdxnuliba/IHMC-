package us.ihmc.simulationConstructionSetTools.tools;

import static org.junit.Assert.assertNotNull;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotToolsTest
{
   private RandomFloatingRevoluteJointChain getRandomFloatingChain()
   {
      Random random = new Random();

      Vector3D[] jointAxes = {new Vector3D(1.0, 0.0, 0.0)};
      RandomFloatingRevoluteJointChain randomFloatingChain = new RandomFloatingRevoluteJointChain(random, jointAxes);

      randomFloatingChain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);

      return randomFloatingChain;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testScsRobotFromInverseDynamicsRobotModel()
   {
      SCSRobotFromInverseDynamicsRobotModel scsRobotFromInverseDynamicsRobotModel = new RobotTools.SCSRobotFromInverseDynamicsRobotModel("robot",
                                                                                       getRandomFloatingChain().getRootJoint());

      assertNotNull(scsRobotFromInverseDynamicsRobotModel);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAddScsJointUsingIDJoint()
   {
      Joint scsRootJoint = RobotTools.addSCSJointUsingIDJoint(getRandomFloatingChain().getRootJoint(), new Robot("robot"), true);

      assertNotNull(scsRootJoint);
   }
}
