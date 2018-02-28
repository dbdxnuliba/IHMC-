package us.ihmc.atlas;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class AtlasToeSCSGroundTrack {

	public static void main(String[] args) {
		DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

		AtlasRobotModel atlasRobotModel = new AtlasToeRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_TOE_JOINT, RobotTarget.SCS, false);

		final double groundHeight = 0.0;
		GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

		DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, atlasRobotModel.getSimulateDT());

		scsInitialSetup.setDrawGroundProfile(true);
		scsInitialSetup.setInitializeEstimatorToActual(true);

		double initialYaw = 0.3;
		DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = atlasRobotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

		boolean useVelocityAndHeadingScript = true;
		boolean cheatWithGroundHeightAtForFootstep = false;

		DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
				useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, atlasRobotModel);

	}
}
