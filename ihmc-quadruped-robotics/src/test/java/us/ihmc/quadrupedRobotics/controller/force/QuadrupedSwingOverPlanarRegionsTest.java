package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.WideWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;

@Tag("humanoid-rough-terrain")
public class QuadrupedSwingOverPlanarRegionsTest
{



   @Test
   public void testSwingOverPlanarRegions() throws SimulationExceededMaximumTimeException
   {
      double stepDuration = 0.33;
      double stepLength = 0.3;
      double stepWidth = 0.25;
      int steps = 10;

      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();
      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, 100000);
      scsParameters.setUseAutoGroundGraphics(false);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), scsParameters);

      QuadrupedSwingOverPlanarRegionsVisualizer swingOverPlanarRegionsVisualizer = new QuadrupedSwingOverPlanarRegionsVisualizer(scs, registry,
                                                                                       yoGraphicsListRegistry);
      QuadrupedPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander = swingOverPlanarRegionsVisualizer.getSwingOverPlanarRegionsTrajectoryExpander();



      scs.setDT(0.0001, 1);
      scs.addYoVariableRegistry(registry);
      scs.setGroundVisible(false);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.25);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);
      scs.startOnAThread();

      FramePoint3D stanceFootPosition = new FramePoint3D();
      QuadrantDependentList<FramePoint3D> footPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), robotQuadrant.getEnd().negateIfHindEnd(0.5), robotQuadrant.getSide().negateIfRightSide(0.25), 0.0);
         footPositions.put(robotQuadrant, footPosition);
      }

      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();

      stanceFootPosition.set(0.0, -stepWidth, 0.0);
      swingEndPosition.set(0.0, stepWidth, 0.0);


      double hindStart = footPositions.get(RobotQuadrant.HIND_LEFT).getX();
      double frontStart = footPositions.get(RobotQuadrant.FRONT_LEFT).getX();
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

         double footstepY = robotSide.negateIfRightSide(stepWidth);
         double footstepX = stepLength * i;

         RobotQuadrant frontQuadrant = RobotQuadrant.getQuadrant(RobotEnd.FRONT, robotSide);
         RobotQuadrant hindQuadrant = RobotQuadrant.getQuadrant(RobotEnd.HIND, robotSide.getOppositeSide());

         Point3D frontLocation = new Point3D(frontStart + footstepX, footstepY, 0.0);
         Point3D hindLocation = new Point3D(hindStart + footstepX, -footstepY, 0.0);


         swingStartPosition.set(footPositions.get(frontQuadrant));
         swingEndPosition.set(frontLocation);

         double maxSpeedDimensionless = Double.NaN;

            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, planarRegionsList);

         PrintTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         PrintTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);



         swingStartPosition.set(footPositions.get(hindQuadrant));
         swingEndPosition.set(hindLocation);

         maxSpeedDimensionless = Double.NaN;

            maxSpeedDimensionless = swingOverPlanarRegionsVisualizer.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, planarRegionsList);

         PrintTools.info("Step " + i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         PrintTools.info("Foot: " + robotSide + "  X: " + footstepX + "  Y: " + footstepY);


         double maxSpeed = maxSpeedDimensionless / stepDuration;


         footPositions.get(frontQuadrant).set(frontLocation);
         footPositions.get(hindQuadrant).set(hindLocation);
      }

      ThreadTools.sleepForever();


   }

}
