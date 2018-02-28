package us.ihmc.atlas;

import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasToeRobotModel extends AtlasRobotModel {

	public AtlasToeRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless){
		super(atlasVersion, target, headless);

	}

	@Override
	public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight,
			double initialYaw) {
		return new AtlasSimSetup(groundHeight, initialYaw);
	}

	
	
	
	
	
	
	
	
	
	
	
	private class AtlasSimSetup extends AtlasSimInitialSetup {

		public AtlasSimSetup(double groundHeight, double initialYaw) {
			super(groundHeight, initialYaw);
			System.out.println("ground toe height"+groundHeight);
		}
		
		private boolean robotInitialized = false;

		@Override
		public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap) {

			System.out.println("atlas toe initial setup called");
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
