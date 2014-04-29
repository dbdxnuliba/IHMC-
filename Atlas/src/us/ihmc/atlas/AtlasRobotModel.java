package us.ihmc.atlas;

import com.jme3.math.Transform;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.*;
import us.ihmc.atlas.physics.AtlasPhysicsEngineConfiguration;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.iRobot.model.iRobotHandModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import java.io.InputStream;

public class AtlasRobotModel implements DRCRobotModel
{
   private final AtlasRobotVersion selectedVersion;
   
   private final boolean runningOnRealRobot;
   private JaxbSDFLoader headlessLoader;
   private JaxbSDFLoader loader;

   public AtlasRobotModel(AtlasRobotVersion atlasVersion, boolean runningOnRealRobot)
   {
      selectedVersion = atlasVersion;
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return new AtlasArmControllerParameters(runningOnRealRobot);
   }

   @Override
   public WalkingControllerParameters getWalkingControlParameters()
   {
      return new AtlasWalkingControllerParameters(runningOnRealRobot);
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT)
   {
      return new AtlasStateEstimatorParameters(runningOnRealRobot, estimatorDT);
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return new AtlasPhysicalProperties();
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return new AtlasJointMap(selectedVersion);
   }

   public boolean hasIRobotHands()
   {
      return selectedVersion.getHandModel() == DRCHandType.IROBOT;
   }

   public boolean hasArmExtensions()
   {
      return selectedVersion.hasArmExtensions();
   }

   public boolean hasHookHands()
   {
      return selectedVersion.getHandModel() == DRCHandType.HOOK;
   }

   public DRCHandType getHandType()
   {
      return selectedVersion.getHandModel();
   }

   @Override
   public String getModelName()
   {
      return "atlas";
   }

   public AtlasRobotVersion getAtlasVersion()
   {
      return selectedVersion;
   }

   public String getSdfFile()
   {
      return selectedVersion.getSdfFile();
   }

   public String[] getResourceDirectories()
   {
      return selectedVersion.getResourceDirectories();
   }

   public InputStream getSdfFileAsStream()
   {
      return selectedVersion.getSdfFileAsStream();
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return selectedVersion.getOffsetFromWrist(side);
   }

   @Override
   public String toString()
   {
      return selectedVersion.toString();
   }

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new AtlasSimInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return new AtlasRobotMultiContactControllerParameters();
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure( SDFRobot robotModel )
   {
      return new AtlasPhysicsEngineConfiguration(getJointMap(),robotModel);
   }

   @Override
   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new AtlasContactPointParamaters(selectedVersion,getJointMap(),addLoadsOfContactPoints,addLoadsOfContactPointsToFeetOnly);
   }

   @Override
   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return null;
   }

   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, double controlDT, boolean runningOnRealRobot)
   {
      throw new RuntimeException("Implement me!");
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      AtlasDampingParameters.setDampingParameters(simulatedRobot, getHandType(), getJointMap());
   }

   @Override
   public HandModel getHandModel()
   {
      if(selectedVersion.getHandModel() == DRCHandType.IROBOT)
		   return new iRobotHandModel();
	   else
		   return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return new AtlasDrivingControllerParameters();
   }

   @Override
   public JaxbSDFLoader getJaxbSDFLoader(boolean headless)
   {
      if(headless)
      {
         if(headlessLoader == null)
         {
            this.headlessLoader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), headless);
         }
         return headlessLoader;
      }
      
      if(loader == null)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), headless);
      }
      return loader;
   }
}
