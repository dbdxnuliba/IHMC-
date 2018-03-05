package us.ihmc.atlas.toeModelWalking;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasToeRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.straightLegWalking.AvatarStraightLegWalkingTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasToeModelWalkingTest extends AvatarStraightLegWalkingTest {

	private final AtlasRobotModel atlasRobotModel = new AtlasToeRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_TOE_JOINT, RobotTarget.SCS, false);

	@Override
	@ContinuousIntegrationTest(estimatedDuration =  20.0, categoriesOverride = {IntegrationCategory.FAST})
	@Test(timeout = 200000)
	public void testForwardWalking() throws SimulationExceededMaximumTimeException
	{
		super.testForwardWalking();
	}

	@Override
	@ContinuousIntegrationTest(estimatedDuration =  20.0, categoriesOverride = {IntegrationCategory.FAST})
	@Test(timeout = 200000)
	public void testSlowerWalking() throws SimulationExceededMaximumTimeException
	{
		super.testSlowerWalking();
	}

	@Override
	@ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
	@Test(timeout = 200000)
	public void testWalkingOverCinderBlockField() throws Exception
	{
		super.testWalkingOverCinderBlockField();
	}

	@Override
	@ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.FAST})
	@Test(timeout = 200000)
	public void testWalkingOverStairs() throws Exception
	{
		super.testWalkingOverStairs();
	}

	@ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
	@Test(timeout = 200000)
	public void testDropOffsWhileWalking() throws Exception
	{
		double stepDownHeight = 0.08;
		super.testDropOffsWhileWalking(stepDownHeight);
	}

	@ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.FAST})
	@Test(timeout = 200000)
	public void testSteppingDown() throws Exception
	{
		double stepDownHeight = 0.12;
		super.testSteppingDown(stepDownHeight, 0.30, 1);
	}

	@ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
	@Test(timeout = 200000)
	public void testSteppingDownEveryTime() throws Exception
	{
		double stepLength = 0.35;
		double stepDownHeight = 0.15;
		super.testSteppingDown(stepDownHeight, stepLength, 0);
	}

	/*
  @ContinuousIntegrationTest(estimatedDuration =  167.7, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
  @Test(timeout = 200000)
  public void testRandomHeightField() throws Exception
  {
     double maxStepIncrease = 0.07;
     double maxStepHeight = 0.04;
     double minStepHeight = -0.12;
     super.testRandomHeightField(maxStepHeight, minStepHeight, maxStepIncrease);
  }
	 */

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


	public static void main(String[] args) throws Exception
	{
		AtlasToeModelWalkingTest test = new AtlasToeModelWalkingTest();
		test.testForwardWalking();
	}

}
