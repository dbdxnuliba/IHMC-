package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPawPlannerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPointFootSnapperParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkModuleParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;

public class GenericQuadrupedNetworkProcessor extends QuadrupedNetworkProcessor
{
   public GenericQuadrupedNetworkProcessor(DomainFactory.PubSubImplementation pubSubImplementation, QuadrupedNetworkModuleParameters networkModuleParameters)
   {
      this(new GenericQuadrupedModelFactory(),
           new GenericQuadrupedPhysicalProperties().getFeetGroundContactPoints(),
           new GenericQuadrupedPawPlannerParameters(),
           new GenericQuadrupedXGaitSettings(),
           new GenericQuadrupedPointFootSnapperParameters(),
           pubSubImplementation,
           networkModuleParameters);
   }

   public GenericQuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, QuadrantDependentList<ArrayList<Point2D>> groundContactPoints,
                                           PawStepPlannerParametersBasics pawPlannerParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                           PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation,
                                           QuadrupedNetworkModuleParameters networkModuleParameters)
   {
      super(robotModel,
            networkModuleParameters,
            groundContactPoints, pawPlannerParameters,
            xGaitSettings,
            pointFootSnapperParameters,
            pubSubImplementation);
   }

   public static void main(String[] args)
   {
      QuadrupedNetworkModuleParameters networkModuleParameters = new QuadrupedNetworkModuleParameters();

      networkModuleParameters.enableFootstepPlanningModule(true);
      networkModuleParameters.enableStepTeleopModule(true);

      new GenericQuadrupedNetworkProcessor(DomainFactory.PubSubImplementation.INTRAPROCESS, networkModuleParameters);
   }
}
