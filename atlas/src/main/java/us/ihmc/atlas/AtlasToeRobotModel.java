package us.ihmc.atlas;

import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasToeRobotModel extends AtlasRobotModel {

	public AtlasToeRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless){
		super(atlasVersion, target, headless);

	}

	@Override
	public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight,
			double initialYaw) {
		return new AtlasSimSetup(groundHeight, initialYaw);
	}

	@Override
	public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
	{
		boolean enableTorqueVelocityLimits = false;
		HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = new HumanoidFloatingRootJointRobot(super.getRobotDescription(), super.getJointMap(), enableJointDamping,
				enableTorqueVelocityLimits);

		// adding ground contact points for the ankle roll joint.
		for (RobotSide robotSide : RobotSide.values)
			humanoidFloatingRootJointRobot.addFootGroundContactPoints(robotSide,humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(super.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)));

		return humanoidFloatingRootJointRobot;
	}


	private class AtlasSimSetup extends AtlasSimInitialSetup {

		public AtlasSimSetup(double groundHeight, double initialYaw) {
			super(groundHeight, initialYaw);
		}

		private boolean robotInitialized = false;

		@Override
		public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap) {

			if(!robotInitialized)
			{
				robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.TOE_PITCH)).setQ(0.0); 	//l_leg_toe
				robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.TOE_PITCH)).setQ(0.0); 	//r_leg_toe
				robotInitialized = true;
			}

			super.initializeRobot(robot, jointMap);

		}
	}
}
