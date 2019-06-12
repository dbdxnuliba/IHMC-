package us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.AStarPawStepPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.VisibilityGraphWithAStarPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.YoPawStepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn.QuadrupedSplineWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn.QuadrupedVisGraphWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawStepPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class PawPlanningController extends QuadrupedToolboxController
{
   private final YoEnum<PawStepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, PawStepPlannerType.class);
   private final EnumMap<PawStepPlannerType, BodyPathAndPawPlanner> plannerMap = new EnumMap<>(PawStepPlannerType.class);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicReference<PawStepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private PawStepPlannerStart start;

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final PawStepPlannerParametersBasics pawPlannerParameters;
   private final AtomicLong robotTimestampNanos = new AtomicLong();
   private final YoDouble robotTimestamp = new YoDouble("robotTimestamp", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoDouble timeout = new YoDouble("toolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);
   private final YoDouble obstacleAvoidanceRadius = new YoDouble("obstacleAvoidanceRadius", registry);
   private final YoBoolean avoidObstaclesInSwing = new YoBoolean("avoidObstaclesInSwing", registry);

   private final QuadrupedPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   public PawPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, VisibilityGraphsParametersBasics defaultVisibilityGraphParameters,
                                PawStepPlannerParametersBasics pawPlannerParameters, PointFootSnapperParameters pointFootSnapperParameters,
                                OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      this.pawPlannerParameters = pawPlannerParameters;
      this.visibilityGraphParameters = defaultVisibilityGraphParameters;

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      new YoPawStepPlannerParameters(pawPlannerParameters, registry);
      new YoVisibilityGraphParameters(registry, visibilityGraphParameters);

      obstacleAvoidanceRadius.set(0.05);
      swingOverPlanarRegionsTrajectoryExpander = new QuadrupedPlanarRegionsTrajectoryExpander(registry, graphicsListRegistry);

      if (robotDataReceiver != null)
      {
         plannerMap.put(PawStepPlannerType.SIMPLE_PATH_TURN_WALK_TURN,
                        new QuadrupedSplineWithTurnWalkTurnPlanner(xGaitSettings, robotTimestamp, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(), null, registry));
         plannerMap.put(PawStepPlannerType.VIS_GRAPH_WITH_TURN_WALK_TURN,
                        new QuadrupedVisGraphWithTurnWalkTurnPlanner(xGaitSettings, visibilityGraphParameters, robotTimestamp, pointFootSnapperParameters,
                                                                     robotDataReceiver.getReferenceFrames(), null, registry));
      }
      plannerMap.put(PawStepPlannerType.A_STAR,
                     AStarPawStepPlanner.createPlanner(this.pawPlannerParameters, xGaitSettings, null, registry));
      plannerMap.put(PawStepPlannerType.VIS_GRAPH_WITH_A_STAR, new VisibilityGraphWithAStarPawPlanner(this.pawPlannerParameters, xGaitSettings,
                                                                                                      visibilityGraphParameters, graphicsListRegistry, registry));
      activePlanner.set(PawStepPlannerType.SIMPLE_PATH_TURN_WALK_TURN);

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      for (PawStepPlannerType plannerKey : plannerMap.keySet())
      {
         BodyPathAndPawPlanner planner = plannerMap.get(plannerKey);
         planner.setGroundPlane(message);
      }
   }

   public void processSupportRegionParameters(QuadrupedSupportPlanarRegionParametersMessage message)
   {
      for (PawStepPlannerType plannerKey : plannerMap.keySet())
      {
         WaypointsForPawStepPlanner planner = plannerMap.get(plannerKey).getWaypointPathPlanner();
         if (planner != null)
            planner.setFallbackRegionSize(message.getInsideSupportRegionSize());
      }
   }

   public void processFootstepPlannerParametersPacket(PawStepPlannerParametersPacket packet)
   {
      pawPlannerParameters.set(packet);
   }

   public void processVisibilityGraphParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      visibilityGraphParameters.set(packet);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processRobotTimestamp(long timestampInNanos)
   {
      this.robotTimestampNanos.set(timestampInNanos);
   }

   public void processPawPlanningRequest(PawStepPlanningRequestPacket footstepPlanningRequestPacket)
   {
      latestRequestReference.set(footstepPlanningRequestPacket);
   }

   @Override
   public boolean initializeInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      isDone.set(false);

      PawStepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      if (request.getObstacleAvoidanceRadius() != -1.0)
         obstacleAvoidanceRadius.set(request.getObstacleAvoidanceRadius());
      avoidObstaclesInSwing.set(request.getAvoidObstaclesInSwing());

      planId.set(request.getPlannerRequestId());
      if (request.getRequestedPawPlannerType() >= 0)
         activePlanner.set(PawStepPlannerType.fromByte(request.getRequestedPawPlannerType()));

      PlanarRegionsListMessage planarRegionsListMessage = request.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         if (planarRegionsList.isEmpty())
            this.planarRegionsList = Optional.empty();
         else
            this.planarRegionsList = Optional.of(planarRegionsList);
      }

      BodyPathAndPawPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegionsList(planarRegionsList.get());
      }
      else
      {
         planner.setPlanarRegionsList(null);
      }

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getGoalPositionInWorld(), request.getGoalOrientationInWorld());

      PawStepPlannerStart start = new PawStepPlannerStart();
      PawStepPlannerGoal goal = new PawStepPlannerGoal();

      PawStepPlannerTargetType targetType = PawStepPlannerTargetType.fromByte(request.getStartTargetType());
      if (targetType == PawStepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getBodyPositionInWorld(), request.getBodyOrientationInWorld());
         start.setStartPose(initialPose);
         start.setStartType(targetType);
      }
      else
      {
         start.setPawStartPosition(RobotQuadrant.FRONT_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontLeftPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.FRONT_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontRightPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.HIND_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindLeftPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.HIND_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindRightPositionInWorld()));
         start.setStartType(PawStepPlannerTargetType.FOOTSTEPS);
      }

      start.setInitialQuadrant(RobotQuadrant.fromByte(request.getInitialStepRobotQuadrant()));
      goal.setGoalPose(goalPose);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0.0 && Double.isFinite(horizonLength))
         planner.setPlanningHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         this.timeout.set(timeout);
         planner.setTimeout(timeout);
      }
      else
      {
         planner.setTimeout(Double.POSITIVE_INFINITY);
      }

      double bestEffortTimeout = request.getBestEffortTimeout();
      if (bestEffortTimeout > 0.0 && Double.isFinite(bestEffortTimeout))
      {
         planner.setBestEffortTimeout(bestEffortTimeout);
      }
      else
      {
         planner.setBestEffortTimeout(0.0);
      }

      planner.setStart(start);
      planner.setGoal(goal);

      return true;
   }

   @Override
   public void updateInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      BodyPathAndPawPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      reportMessage(packStatus(PawStepPlannerStatus.PLANNING_PATH));

      PawStepPlanningResult status = planner.planPath();

      BodyPathPlan bodyPathPlan = null;
      if (status.validForExecution())
      {
         bodyPathPlan = planner.getPathPlan();
         reportMessage(packStatus(PawStepPlannerStatus.PLANNING_STEPS));
         reportMessage(packPathResult(bodyPathPlan, status));

         status = planner.plan();
      }

      PawStepPlan pawStepPlan;
      if (status.validForExecution())
         pawStepPlan = planner.getPlan();
      else
         pawStepPlan = null;

      reportMessage(packStepResult(pawStepPlan, bodyPathPlan, status));

      finishUp();
   }

   public void finishUp()
   {
      if (DEBUG)
         PrintTools.info("Finishing up the planner");
      plannerMap.get(activePlanner.getEnumValue()).cancelPlanning();
      reportMessage(packStatus(PawStepPlannerStatus.IDLE));
      isDone.set(true);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlannerStatusMessage packStatus(PawStepPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
   }

   private BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, PawStepPlanningResult status)
   {
      if (DEBUG)
      {
         PrintTools.info("Finished planning path. Result: " + status);
      }

      BodyPathPlanMessage result = new BodyPathPlanMessage();
      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));

         result.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
         result.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      return result;
   }

   private PawStepPlanningToolboxOutputStatus packStepResult(PawStepPlan pawStepPlan, BodyPathPlan bodyPathPlan, PawStepPlanningResult status)
   {
      PawStepPlanningToolboxOutputStatus result = new PawStepPlanningToolboxOutputStatus();
      if (pawStepPlan == null)
      {
         result.getFootstepDataList().set(new QuadrupedTimedStepListMessage());
      }
      else
      {
         result.getFootstepDataList().set(convertToTimedStepListMessage(pawStepPlan));
         result.getLowLevelPlannerGoal().set(pawStepPlan.getLowLevelPlanGoal());
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      result.setTimeTaken(plannerMap.get(activePlanner.getEnumValue()).getPlanningDuration());

      if (avoidObstaclesInSwing.getBooleanValue())
         addObstacleAvoidanceWaypointsWhenNecessary(result.getFootstepDataList());

      return result;
   }

   private void addObstacleAvoidanceWaypointsWhenNecessary(QuadrupedTimedStepListMessage stepListMessage)
   {
      if (!planarRegionsList.isPresent())
         return;

      swingOverPlanarRegionsTrajectoryExpander.setCollisionSphereRadius(obstacleAvoidanceRadius.getDoubleValue());

      QuadrantDependentList<FramePoint3D> footPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(start.getPawGoalPosition(robotQuadrant));
         footPositions.put(robotQuadrant, footPosition);
      }

      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();

      for (int stepIndex = 0; stepIndex < stepListMessage.getQuadrupedStepList().size(); stepIndex++)
      {
         QuadrupedStepMessage stepMessage = stepListMessage.getQuadrupedStepList().get(stepIndex).getQuadrupedStepMessage();
         RobotQuadrant movingQuadrant = RobotQuadrant.fromByte(stepMessage.getRobotQuadrant());
         swingStartPosition.set(footPositions.get(movingQuadrant));
         swingEndPosition.set(stepMessage.getGoalPosition());

         if (swingOverPlanarRegionsTrajectoryExpander
               .expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, xGaitSettings.getStepGroundClearance(), planarRegionsList.get(), null))
         {
            List<FramePoint3D> expandedWaypoints = swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints();
            for (int waypointIndex = 0; waypointIndex < expandedWaypoints.size(); waypointIndex++)
            {
               stepMessage.getCustomPositionWaypoints().add().set(expandedWaypoints.get(waypointIndex));
            }
         }

         footPositions.get(movingQuadrant).set(swingEndPosition);
      }
   }


   private static QuadrupedTimedStepListMessage convertToTimedStepListMessage(PawStepPlan pawStepPlan)
   {
      if (pawStepPlan == null)
         return null;

      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < pawStepPlan.getNumberOfSteps(); i++)
      {
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(pawStepPlan.getPawStep(i)));
      }

      return QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, false);
   }
}
