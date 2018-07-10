package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringDefinition;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringDefinitionOnConstrainedTrajectory;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringDefinitionToReachingManifold;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringProgressVisualizer;
import us.ihmc.manipulation.planning.exploringSpatial.SpatialNode;
import us.ihmc.manipulation.planning.exploringSpatial.SpatialNodePlotter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyTrajectoryToolboxController extends ToolboxController
{
   private static final boolean VERBOSE = true;
   private static final int DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION = 10;
   private static final int DEFAULT_MAXIMUM_NUMBER_OF_ITERATIONS = 3000;
   private static final int DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE = 1000;
   private static final int DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE = 200;
   private static final int TERMINAL_CONDITION_NUMBER_OF_VALID_INITIAL_GUESSES = 20;

   private final HumanoidKinematicsSolver humanoidKinematicsSolver;

   private final WholeBodyTrajectoryToolboxOutputStatus toolboxSolution;

   private List<RigidBodyExplorationConfigurationCommand> rigidBodyCommands = null;
   private List<WaypointBasedTrajectoryCommand> trajectoryCommands = null;
   private List<ReachingManifoldCommand> manifoldCommands = null;

   /*
    * YoVariables
    */
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
   private final YoInteger currentNumberOfIterations = new YoInteger("currentNumberOfIterations", registry);
   private final YoInteger terminalConditionNumberOfValidInitialGuesses = new YoInteger("terminalConditionNumberOfValidInitialGuesses", registry);

   private final YoInteger maximumExpansionSize = new YoInteger("maximumExpansionSize", registry);
   private final YoInteger desiredNumberOfInitialGuesses = new YoInteger("desiredNumberOfInitialGuesses", registry);

   private YoInteger numberOfIterationForShortcutOptimization = new YoInteger("numberOfIterationForShortcutOptimization", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final YoBoolean isValidNode = new YoBoolean("isValidNode", registry);

   /*
    * Visualizer
    */
   private boolean visualize;

   private SpatialNode visualizedNode;

   private final KinematicsToolboxOutputConverter configurationConverter;
   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();
   private final AtomicReference<RobotConfigurationData> currentRobotConfigurationDataReference = new AtomicReference<>(null);

   private FullHumanoidRobotModel visualizedFullRobotModel;

   private ExploringProgressVisualizer treeStateVisualizer;

   private SpatialNodePlotter nodePlotter;

   private final SideDependentList<YoFramePoseUsingYawPitchRoll> endeffectorPose = new SideDependentList<>();

   private final SideDependentList<YoGraphicCoordinateSystem> endeffectorFrame = new SideDependentList<>();

   private final List<SpatialNode> path = new ArrayList<>();

   /**
    * Toolbox state
    */
   // TODO will be replaced with activated manager.
   private final YoEnum<CWBToolboxState> state = new YoEnum<>("state", registry, CWBToolboxState.class);

   private enum CWBToolboxState
   {
      DO_NOTHING, FIND_INITIAL_GUESS, EXPAND_TREE, SHORTCUT_PATH, GENERATE_MOTION
   }

   private final CommandInputManager commandInputManager;

   private ExploringDefinition spatialDefinition;

   private WholeBodyTrajectoryToolboxManager activatedManager;
   private InitialGuessManager initialGuessManager;
   private ExploringManager exploringManager;
   // TODO : shortcut manager will make simpler?

   public WholeBodyTrajectoryToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager, YoVariableRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      super(statusOutputManager, registry);
      this.commandInputManager = commandInputManager;

      visualizedFullRobotModel = fullRobotModel;
      isDone.set(false);

      this.visualize = visualize;
      if (visualize)
      {
         treeStateVisualizer = new ExploringProgressVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsListRegistry, registry);
      }
      else
      {
         treeStateVisualizer = null;
      }
      state.set(CWBToolboxState.DO_NOTHING);

      for (RobotSide robotSide : RobotSide.values)
      {
         endeffectorPose.put(robotSide, new YoFramePoseUsingYawPitchRoll("" + robotSide + "endeffectorPose", ReferenceFrame.getWorldFrame(), registry));

         endeffectorFrame.put(robotSide, new YoGraphicCoordinateSystem("" + robotSide + "endeffectorPoseFrame", endeffectorPose.get(robotSide), 0.25));
         endeffectorFrame.get(robotSide).setVisible(true);
         yoGraphicsListRegistry.registerYoGraphic("" + robotSide + "endeffectorPoseViz", endeffectorFrame.get(robotSide));
      }

      numberOfIterationForShortcutOptimization.set(DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION);
      maximumNumberOfIterations.set(DEFAULT_MAXIMUM_NUMBER_OF_ITERATIONS);
      terminalConditionNumberOfValidInitialGuesses.set(TERMINAL_CONDITION_NUMBER_OF_VALID_INITIAL_GUESSES);

      humanoidKinematicsSolver = new HumanoidKinematicsSolver(drcRobotModel, yoGraphicsListRegistry, registry);

      toolboxSolution = new WholeBodyTrajectoryToolboxOutputStatus();
      toolboxSolution.setDestination(-1);

      configurationConverter = new KinematicsToolboxOutputConverter(drcRobotModel);
   }

   @Override
   protected void updateInternal() throws InterruptedException, ExecutionException
   {
      currentNumberOfIterations.increment();

      // ************************************************************************************************************** //
      switch (state.getEnumValue())
      {
      case DO_NOTHING:

         break;
      case FIND_INITIAL_GUESS:

         activatedManager = initialGuessManager;
         findInitialGuess();

         break;
      case EXPAND_TREE:

         activatedManager = exploringManager;
         expandingTree();

         break;
      case SHORTCUT_PATH:

         shortcutPath();

         break;
      case GENERATE_MOTION:

         generateMotion();

         break;
      }
      // ************************************************************************************************************** //

      // ************************************************************************************************************** //

      updateVisualizerRobotConfiguration();
      updateVisualizers();
      updateYoVariables();

      // ************************************************************************************************************** //
      if (currentNumberOfIterations.getIntegerValue() == maximumNumberOfIterations.getIntegerValue())
      {
         terminateToolboxController();
      }
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, int planningResult)
   {
      outputStatusToPack.setPlanningResult(planningResult);
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, List<SpatialNode> path)
   {
      if (outputStatusToPack.getPlanningResult() == 4)
      {
         MessageTools.copyData(path.stream().map(SpatialNode::getConfiguration).toArray(size -> new KinematicsToolboxOutputStatus[size]),
                               outputStatusToPack.getRobotConfigurations());
         outputStatusToPack.getTrajectoryTimes().reset();
         outputStatusToPack.getTrajectoryTimes().add(path.stream().mapToDouble(SpatialNode::getTime).toArray());
      }
      else
      {
         if (VERBOSE)
            PrintTools.info("Planning has Failed.");
      }
   }

   /**
    * state == GENERATE_MOTION
    */
   private void generateMotion()
   {
      /*
       * terminate generateMotion.
       */
      if (true)
      {
         PrintTools.info("inho delete next line");
         nodePlotter.saveNodes();
         setOutputStatus(toolboxSolution, 4);
         setOutputStatus(toolboxSolution, path);

         terminateToolboxController();
      }
   }

   /**
    * state = SHORTCUT_PATH
    */
   private void shortcutPath()
   {
      // smoothing over one mile stone node.
      for (int i = 0; i < numberOfIterationForShortcutOptimization.getIntegerValue(); i++)
         if (updateShortcutPath(path) < 0.001)
            break;

      // plotting final result.
      for (int i = 0; i < path.size(); i++)
         nodePlotter.update(path.get(i), 3);

      /*
       * terminate state
       */
      state.set(CWBToolboxState.GENERATE_MOTION);
   }

   /**
    * state == EXPAND_TREE
    */
   private void expandingTree()
   {
      handleManager();

      if (activatedManager.isDone() == true)
      {
         if (activatedManager.hasFail())
         {
            if(VERBOSE)
               PrintTools.info("has Fail");
            setOutputStatus(toolboxSolution, 2);
            activatedManager.terminalManager();
            terminateToolboxController();
         }
         else
         {
            if(VERBOSE)
               PrintTools.info("success");
            state.set(CWBToolboxState.SHORTCUT_PATH);
            //SHORTCUT_PATH.initialize();
            nodePlotter.update(exploringManager.getPath(), 2);
            path.addAll(exploringManager.getPath());
            activatedManager.terminalManager();
         }
      }

   }

   private void findInitialGuess()
   {
      handleManager();

      if (activatedManager.isDone() == true)
      {
         if (activatedManager.hasFail())
         {
            if(VERBOSE)
               PrintTools.info("has Fail");
            setOutputStatus(toolboxSolution, 1);
            activatedManager.terminalManager();
            terminateToolboxController();
         }
         else
         {
            if(VERBOSE)
               PrintTools.info("success " + initialGuessManager.getValidInitialGuesses().size());
            state.set(CWBToolboxState.EXPAND_TREE);
            exploringManager.initialize();
            exploringManager.setInitialGuesses(initialGuessManager.getValidInitialGuesses());
            activatedManager.terminalManager();
         }
      }
   }

   private void handleManager()
   {
      SpatialNode desiredNode = activatedManager.createDesiredNode();

      if (desiredNode != null)
      {
         updateValidity(desiredNode);

         activatedManager.putDesiredNode(desiredNode);

         visualizedNode = desiredNode;

         nodePlotter.update(desiredNode, 1);
      }

      activatedManager.update();
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);

      /*
       * bring control parameters from request.
       */
      boolean success = updateConfiguration();
      if (!success)
      {
         return false;
      }

      if (!commandInputManager.isNewCommandAvailable(RigidBodyExplorationConfigurationCommand.class))
      {
         return false;
      }

      rigidBodyCommands = commandInputManager.pollNewCommands(RigidBodyExplorationConfigurationCommand.class);

      // ******************************************************************************** //
      // Convert command into WholeBodyTrajectoryToolboxData.
      // ******************************************************************************** //
      if (VERBOSE)
         PrintTools.info("initialize CWB toolbox");

      configurationConverter.updateFullRobotModel(initialConfiguration);

      // ExploringDefinition spatialDefinition;
      if (trajectoryCommands != null)
      {
         spatialDefinition = new ExploringDefinitionOnConstrainedTrajectory(trajectoryCommands, rigidBodyCommands);

         initialGuessManager = new InitialGuessManager(spatialDefinition, desiredNumberOfInitialGuesses.getIntegerValue(),
                                                       terminalConditionNumberOfValidInitialGuesses.getIntegerValue());
         exploringManager = new ExploringManager(spatialDefinition, DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);

         state.set(CWBToolboxState.FIND_INITIAL_GUESS);

         initialGuessManager.initialize();
      }
      else if (manifoldCommands != null)
      {
         spatialDefinition = new ExploringDefinitionToReachingManifold(null, manifoldCommands, rigidBodyCommands);

         exploringManager = new ExploringManager(spatialDefinition, DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);

         state.set(CWBToolboxState.EXPAND_TREE);

         exploringManager.initialize();
      }
      else
      {
         return false;
      }

      nodePlotter = new SpatialNodePlotter(spatialDefinition, visualize);
      return true;
   }

   private boolean updateConfiguration()
   {
      int newMaxExpansionSize = -1;
      int newNumberOfInitialGuesses = -1;
      KinematicsToolboxOutputStatus newInitialConfiguration = null;

      // pull commands. configuration, trajectory, reaching.
      if (commandInputManager.isNewCommandAvailable(WholeBodyTrajectoryToolboxConfigurationCommand.class))
      {
         WholeBodyTrajectoryToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(WholeBodyTrajectoryToolboxConfigurationCommand.class);

         trajectoryCommands = null;
         manifoldCommands = null;
         if (commandInputManager.isNewCommandAvailable(WaypointBasedTrajectoryCommand.class))
         {
            trajectoryCommands = commandInputManager.pollNewCommands(WaypointBasedTrajectoryCommand.class);
            if (trajectoryCommands.size() < 1)
               return false;
         }
         else if (commandInputManager.isNewCommandAvailable(ReachingManifoldCommand.class))
         {
            manifoldCommands = commandInputManager.pollNewCommands(ReachingManifoldCommand.class);
            if (manifoldCommands.size() < 1)
               return false;
         }
         else
         {
            if (VERBOSE)
               PrintTools.info("wrong type received");
            return false;
         }

         newMaxExpansionSize = command.getMaximumExpansionSize();
         newNumberOfInitialGuesses = command.getNumberOfInitialGuesses();

         if (command.hasInitialConfiguration())
         {
            newInitialConfiguration = command.getInitialConfiguration();
         }
      }

      // toolbox setup parameters.
      if (newMaxExpansionSize > 0)
         maximumExpansionSize.set(newMaxExpansionSize);
      else
         maximumExpansionSize.set(DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);

      if (newNumberOfInitialGuesses > 0)
         desiredNumberOfInitialGuesses.set(newNumberOfInitialGuesses);
      else
         desiredNumberOfInitialGuesses.set(DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE);

      // set initial configuration of full robot model in toolbox.
      if (newInitialConfiguration != null)
      {
         initialConfiguration.set(newInitialConfiguration);
         return true;
      }

      RobotConfigurationData currentRobotConfiguration = currentRobotConfigurationDataReference.getAndSet(null);
      if (currentRobotConfiguration == null)
      {
         return false;
      }

      initialConfiguration.getDesiredRootOrientation().set(currentRobotConfiguration.getRootOrientation());
      initialConfiguration.getDesiredRootTranslation().set(currentRobotConfiguration.getRootTranslation());

      initialConfiguration.setJointNameHash(currentRobotConfiguration.getJointNameHash());
      MessageTools.copyData(currentRobotConfiguration.getJointAngles(), initialConfiguration.getDesiredJointAngles());

      if (VERBOSE)
         PrintTools.info("update config done");
      return true;
   }

   private void terminateToolboxController()
   {
      toolboxSolution.setDestination(PacketDestination.BEHAVIOR_MODULE.ordinal());

      reportMessage(toolboxSolution);

      //nodePlotter.closeAll();
      state.set(CWBToolboxState.DO_NOTHING);

      double totalTime = initialGuessManager.getComputingTime() + exploringManager.getComputingTime();

      if (VERBOSE)
      {
         PrintTools.info("===========================================");
         PrintTools.info("initialGuessComputationTime is " + initialGuessManager.getComputingTime());
         PrintTools.info("treeExpansionComputationTime is " + exploringManager.getComputingTime());
         //PrintTools.info("shortcutPathComputationTime is " + shortcutPathComputationTime.getDoubleValue());
         PrintTools.info("toolbox executing time is " + totalTime + " seconds " + currentNumberOfIterations.getIntegerValue());
         PrintTools.info("===========================================");
      }

      isDone.set(true);
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   /**
    * update validity of input node.
    */
   private boolean updateValidity(SpatialNode node)
   {
      if (node.getParent() != null && node.getParent().getConfiguration() != null)
      {
         humanoidKinematicsSolver.setInitialConfiguration(node.getParent().getConfiguration());
      }
      else
      {
         humanoidKinematicsSolver.setInitialConfiguration(initialConfiguration);
      }

      humanoidKinematicsSolver.initialize();
      humanoidKinematicsSolver.submit(spatialDefinition.createMessages(node));
      /*
       * result
       */
      boolean success = humanoidKinematicsSolver.solve();

      node.setConfiguration(humanoidKinematicsSolver.getSolution());
      node.setValidity(success);

      return success;
   }

   /**
    * set fullRobotModel.
    */
   private void updateVisualizerRobotConfiguration()
   {
      MessageTools.unpackDesiredJointState(visualizedNode.getConfiguration(), visualizedFullRobotModel.getRootJoint(),
                                           FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   /**
    * update visualizers.
    */
   private void updateVisualizers()
   {
      isValidNode.set(visualizedNode.isValid());

      if (visualize && visualizedNode != null)
      {
         treeStateVisualizer.setRecentProgress(spatialDefinition.getExploringProgress(visualizedNode));
         treeStateVisualizer.setDesiredNodeValidity(visualizedNode.isValid());
         treeStateVisualizer.updateVisualizer();
      }
   }

   /**
    * YoVariables.
    */
   private void updateYoVariables()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         endeffectorFrame.get(robotSide).setVisible(true);
         endeffectorFrame.get(robotSide).update();
      }
   }

   /**
    * oneTime shortcut : try to make a shortcut from index to index+2
    */
   private boolean updateShortcutPath(List<SpatialNode> path, int index)
   {
      // check out when index is over the size.
      if (index > path.size() - 3)
      {
         return false;
      }

      SpatialNode nodeDummy = new SpatialNode(path.get(index + 1));

      nodeDummy.setParent(path.get(index));
      nodeDummy.interpolate(path.get(index), path.get(index + 2), 0.5);

      updateValidity(nodeDummy);

      if (nodeDummy.isValid())
      {
         path.get(index + 1).interpolate(path.get(index), path.get(index + 2), 0.5);
         path.get(index + 1).setConfiguration(nodeDummy.getConfiguration());

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * return distance of the paths before and after shortcut.
    */
   private double updateShortcutPath(List<SpatialNode> path)
   {
      ArrayList<SpatialNode> pathBeforeShortcut = new ArrayList<SpatialNode>();

      for (int i = 0; i < path.size(); i++)
      {
         pathBeforeShortcut.add(new SpatialNode(path.get(i)));
      }

      for (int i = 0; i < path.size(); i++)
      {
         if (updateShortcutPath(path, i))
            ;
      }

      double distance = 0.0;
      for (int i = 0; i < path.size(); i++)
      {
         distance += exploringManager.computeDistance(pathBeforeShortcut.get(i), path.get(i));
      }

      return distance / path.size();
   }

   void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      currentRobotConfigurationDataReference.set(newConfigurationData);
   }

   FullHumanoidRobotModel getSolverFullRobotModel()
   {
      return humanoidKinematicsSolver.getDesiredFullRobotModel();
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus takeNextData)
   {
      // TODO Auto-generated method stub
      
   }
}