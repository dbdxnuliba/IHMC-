package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;

@Tag("humanoid-rough-terrain")
public abstract class QuadrupedSwingOverPlanarRegionsTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;


   private static final boolean LOCAL_MODE = false;

   @Test
   public void testSwingOverPlanarRegions() throws SimulationExceededMaximumTimeException
   {
      double stepDuration = 0.33;
      double dwellDuration = 0.2;
      double stepLength = 0.3;
      double stepWidth = 0.25;
      double maxSwingSpeed = 1.0;
      int steps = 10;

      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();
      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      QuadrupedSwingOverPlanarRegionsVisualizer swingOverPlanarRegionsVisualizer = null;
      QuadrupedPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
      if (LOCAL_MODE)
      {
         swingOverPlanarRegionsVisualizer = new QuadrupedSwingOverPlanarRegionsVisualizer(conductor.getScs(), registry,
                                                                                       yoGraphicsListRegistry);
         swingOverPlanarRegionsTrajectoryExpander = swingOverPlanarRegionsVisualizer.getSwingOverPlanarRegionsTrajectoryExpander();
      }
      else
      {
         swingOverPlanarRegionsTrajectoryExpander = new QuadrupedPlanarRegionsTrajectoryExpander(registry, yoGraphicsListRegistry);
      }

      conductor.getScs().getRootRegistry().addChild(registry);
      conductor.getScs().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);


      FramePoint3D stanceFootPosition = new FramePoint3D();
      QuadrantDependentList<FramePoint3D> footPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D();
         footPosition.setToZero(quadrupedTestFactory.getFullRobotModel().getSoleFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         footPositions.put(robotQuadrant, footPosition);
      }

      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();

      stanceFootPosition.set(0.0, -stepWidth, 0.0);
      swingEndPosition.set(0.0, stepWidth, 0.0);

      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();
      stepListMessage.setIsExpressedInAbsoluteTime(false);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
      stepTeleopManager.getXGaitSettings().setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      stepTeleopManager.getXGaitSettings().getTrotMediumTimings().setStepDuration(0.33);
      stepTeleopManager.getXGaitSettings().getTrotMediumTimings().setEndDoubleSupportDuration(0.2);
      stepTeleopManager.getXGaitSettings().getStepDuration();
      double simulationTime = (dwellDuration + stepDuration) * steps + 1.0;

      double currentTime = 0.0;
      double hindStart = footPositions.get(RobotQuadrant.HIND_LEFT).getX();
      double frontStart = footPositions.get(RobotQuadrant.FRONT_LEFT).getX();
      for (int i = 1; i <= steps; i++)
      {
         currentTime += stepDuration + dwellDuration;
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

         double footstepY = robotSide.negateIfRightSide(stepWidth);
         double footstepX = stepLength * i;

         RobotQuadrant frontQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);
         RobotQuadrant hindQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide.getOppositeSide());

         Point3D frontLocation = new Point3D(frontStart + footstepX, footstepY, 0.0);
         Point3D hindLocation = new Point3D(hindStart + footstepX, -footstepY, 0.0);

         QuadrupedTimedStepMessage frontMessage = new QuadrupedTimedStepMessage();
         QuadrupedTimedStepMessage hindMessage = new QuadrupedTimedStepMessage();

         swingStartPosition.set(footPositions.get(frontQuadrant));
         swingEndPosition.set(frontLocation);

         double maxSpeedDimensionless = Double.NaN;

         if (LOCAL_MODE)
         {
            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, planarRegionsList);
         }
         else
         {
            maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, 0.08,
                                                                                                               planarRegionsList, null);
         }

         PrintTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         PrintTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);

         frontMessage.getQuadrupedStepMessage().getGoalPosition().set(frontLocation);
         frontMessage.getQuadrupedStepMessage().setRobotQuadrant(frontQuadrant.toByte());
         frontMessage.getQuadrupedStepMessage().setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         for (int j = 0; j < swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().size(); j++)
            frontMessage.getQuadrupedStepMessage().getCustomPositionWaypoints().add().set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(j));

         swingStartPosition.set(footPositions.get(hindQuadrant));
         swingEndPosition.set(hindLocation);

         maxSpeedDimensionless = Double.NaN;

         if (LOCAL_MODE)
         {
            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, planarRegionsList);
         }
         else
         {
            maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, 0.08,
                                                                                                               planarRegionsList, null);
         }

         PrintTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         PrintTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);

         hindMessage.getQuadrupedStepMessage().getGoalPosition().set(hindLocation);
         hindMessage.getQuadrupedStepMessage().setRobotQuadrant(hindQuadrant.toByte());
         hindMessage.getQuadrupedStepMessage().setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         for (int j = 0; j < swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().size(); j++)
            hindMessage.getQuadrupedStepMessage().getCustomPositionWaypoints().add().set(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(j));


         double maxSpeed = maxSpeedDimensionless / stepDuration;
         if (maxSpeed > maxSwingSpeed)
         {
            double adjustedSwingTime = maxSpeedDimensionless / maxSwingSpeed;
            frontMessage.getTimeInterval().setStartTime(currentTime);
            frontMessage.getTimeInterval().setEndTime(currentTime + stepDuration);
            hindMessage.getTimeInterval().setStartTime(currentTime);
            hindMessage.getTimeInterval().setEndTime(currentTime + stepDuration);

            simulationTime += adjustedSwingTime;
         }
         else
            simulationTime += stepDuration;

         stepListMessage.getQuadrupedStepList().add().set(frontMessage);
         stepListMessage.getQuadrupedStepList().add().set(hindMessage);

         footPositions.get(frontQuadrant).set(frontLocation);
         footPositions.get(hindQuadrant).set(hindLocation);

         currentTime += stepDuration + dwellDuration;
      }

      stepTeleopManager.publishTimedStepListToController(stepListMessage);
      conductor.addTimeLimit(variables.getYoTime(), variables.getYoTime().getDoubleValue() + simulationTime);
      conductor.simulate();

      Point3D rootJointPosition = new Point3D(2.81, 0.0, 0.83);
      Vector3D epsilon = new Vector3D(0.05, 0.05, 0.10);
      Point3D min = new Point3D(rootJointPosition);
      Point3D max = new Point3D(rootJointPosition);
      min.sub(epsilon);
      max.add(epsilon);

      if (LOCAL_MODE)
      {
         ThreadTools.sleepForever();
      }
      else
      {
//         drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));
      }
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();

         LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();

         quadrupedTestFactory.setGroundProfile3D(environment.getTerrainObject3D());
         quadrupedTestFactory.setInitialOffset(new QuadrupedInitialOffsetAndYaw(-0.5, 0.0, 0.0, 0.0));
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
