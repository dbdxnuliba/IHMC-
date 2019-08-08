package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.*;
import org.lwjgl.*;
import org.omg.PortableInterceptor.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commons.lists.*;
import us.ihmc.commons.thread.*;
import us.ihmc.communication.*;
import us.ihmc.communication.net.*;
import us.ihmc.communication.packets.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple4D.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidBehaviors.behaviors.*;
import us.ihmc.humanoidBehaviors.behaviors.primitives.*;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior.*;
import us.ihmc.humanoidBehaviors.taskExecutor.*;
import us.ihmc.humanoidBehaviors.tools.*;
import us.ihmc.humanoidBehaviors.utilities.*;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.communication.subscribers.*;
import us.ihmc.humanoidRobotics.frames.*;
import us.ihmc.log.*;
import us.ihmc.mecano.frames.*;
import us.ihmc.messager.*;
import us.ihmc.messager.MessagerAPIFactory.*;
import us.ihmc.pubsub.subscriber.*;
import us.ihmc.robotModels.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.robotics.taskExecutor.*;
import us.ihmc.ros2.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.tools.thread.*;
import us.ihmc.yoVariables.registry.*;
import us.ihmc.yoVariables.variable.*;

import java.sql.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;

public class SuppaKickBehavior
{
//   private final PipeLine<AbstractBehavior> pipeLine;
   private final BehaviorHelper behaviorHelper;
   private final ActivationReference<Boolean> stepping;
   private final AtomicReference<Boolean> enable;
   private final AtomicInteger footstepsTaken = new AtomicInteger(2);
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
//   private YoDouble yoTime;
   private final YoGraphicsListRegistry yoGraphicsListRegistry  = new YoGraphicsListRegistry();
   private double longTime ;

//   private final PausablePeriodicThread thread;

   //create notifications for actiating behaviors

   private final Notification goToWalk = new Notification();
   private boolean flag1 = false;



   public SuppaKickBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel, Ros2Node ros2Node, double yoTime)
   {
      LogTools.debug("Initializing SearchAndKickBehavior");
//      flag = behaviorHelper.createBooleanActivationReference(API.Walk,false,true);
      this.behaviorHelper = behaviorHelper;

      behaviorHelper.createFootstepStatusCallback(this::acceptFootstepStatus);
      stepping = behaviorHelper.createBooleanActivationReference(API.Stepping, false, true);
      messager.registerTopicListener(API.Abort,this::doOnAbort);
      messager.registerTopicListener(API.Walk, object -> goToWalk.set()); //triggers the notification class set method (like a ping)
//      controllerState = new ROS2Input<>(ros2Node, HighLevelStateChangeStatusMessage.class, robotName, controllerId, initialState, this::acceptStatusChange);

//      ROS2Tools.createCallbackSubscription(ros2Node, ToolboxStateMessage.class, getSubscriberTopicNameGenerator(), s -> receivedPacket(s.takeNextData()));
      enable = messager.createInput(API.Enable, false);

//      thread = new PausablePeriodicThread(this::run, 0.5, getClass().getSimpleName());
//      thread.start();

//      new Thread(() -> {
//         LogTools.info("For yoTime");
//         while(true)
//         {
//            process(yoTime);
//         }
//      }).start();
      ROS2Tools.createCallbackSubscription(ros2Node,
                                           WalkingStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkFootTrajectoryMessage);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           SimulationConstructionSet.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkSimulationMessage);

      ROS2Tools.createCallbackSubscription(ros2Node,
                                           TaskspaceTrajectoryStatusMessage.class,
                                           ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName()),
                                           this::checkTaskspaceTrajectoryMessage);


      behaviorHelper.startScheduledThread(getClass().getSimpleName(), this::doBehavior, 1, TimeUnit.SECONDS);
      CapturePointUpdatable capturePointUpdatable = createCapturePointUpdateable(behaviorHelper,registry, yoGraphicsListRegistry);
      behaviorHelper.addUpdatable(capturePointUpdatable);
//      yoTime = behaviorHelper.getYoTime();
//      this.yoTime = yoTime;
//      longTime = behaviorHelper.getWallTime();



   }

   private CapturePointUpdatable createCapturePointUpdateable(BehaviorHelper testHelper, YoVariableRegistry registry,
                                                              YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      testHelper.createSubscriberFromController(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber::receivedPacket);

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret; // so this is continuously getting updated
   }

   private void checkSimulationMessage(Subscriber<SimulationConstructionSet> message)
   {
      //      System.out.println("Walking Status : " + message.takeNextData().getWalkingStatus());
//      if(message.takeNextData().getWalkingStatus() == 1)
//      {
//         flag1 = true;
//      }
      System.out.println(message.takeNextData().getTime());


   }

//   private void run()
//   {
//
//      process();
//   }
   private void process(double yoTime)
   {
//      System.out.println(behaviorHelper.getLatestControllerState());
//      FootstepDataCommand tmp = new FootstepDataCommand();
//      System.out.println(tmp.getSequenceId());
         longTime = yoTime;
//         System.out.println(longTime);

//      ArrayList<PlanarRegion> combinedRegionsList = new ArrayList<>();


//      synchronized (this)
//      {
//         combinedRegionsList.addAll(customPlanarRegions.values());
//         PlanarRegionsList combinedRegions = new PlanarRegionsList(combinedRegionsList);
//         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedRegions);
//         planarRegionPublisher.publish(message);
//      }
   }

   private void checkTaskspaceTrajectoryMessage(Subscriber<TaskspaceTrajectoryStatusMessage> message)
   {
      System.out.println("Task Space End - effector Name" +message.takeNextData().getEndEffectorNameAsString());
   }

   private void checkFootTrajectoryMessage(Subscriber<WalkingStatusMessage> message)
   {
//      System.out.println("Walking Status : " + message.takeNextData().getWalkingStatus());
      if(message.takeNextData().getWalkingStatus() == 1)
      {
         flag1 = true;
      }
   }


//   public void receivedPacket(ToolboxStateMessage message)
//   {
////      if (DEBUG)
////         LogTools.info("Received a state message.");
////
////      if (toolboxTaskScheduled != null)
////      {
////         return;
////      }
//
//      switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
//      {
//      case WAKE_UP:
//         wakeUp();
//         break;
//      case REINITIALIZE:
//         reinitialize();
//         break;
//      case SLEEP:
//         sleep();
//         break;
//      }
//   }

//   public void wakeUp()
//   {
//      if (toolboxTaskScheduled != null)
//      {
//         if (DEBUG)
//            LogTools.error("This toolbox is already running.");
//         return;
//      }
//
//      if (DEBUG)
//         LogTools.debug("Waking up");
//
//      createToolboxRunnable();
//      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
//      getToolboxController().setFutureToListenTo(toolboxTaskScheduled);
//      reinitialize();
//      receivedInput.set(true);
//   }
//
//   private void reinitialize()
//   {
//      getToolboxController().requestInitialize();
//   }
//
//   public void sleep()
//   {
//
//      if (DEBUG)
//         LogTools.debug("Going to sleep");
//
//      destroyToolboxRunnable();
//
//      if (toolboxTaskScheduled == null)
//      {
//         if (DEBUG)
//            LogTools.error("There is no task running.");
//         return;
//      }
//
//      getToolboxController().setFutureToListenTo(null);
//      toolboxTaskScheduled.cancel(true);
//      toolboxTaskScheduled = null;
//   }
//
   private void doOnAbort(boolean abort)
   {
      if (abort)
      {
         LogTools.info("Abort received. Shutting down threadScheduler.");
         behaviorHelper.shutdownScheduledThread();
      }
   }

   private void acceptFootstepStatus(FootstepStatusMessage footstepStatusMessage)
   {
      if (!enable.get())
         return;

      LogTools.info("acceptFootstepStatus: " + footstepStatusMessage);

      if (footstepStatusMessage.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
      {
         int footstepsTakenSoFar = footstepsTaken.incrementAndGet();
         LogTools.info("Have taken " + footstepsTakenSoFar + " footsteps.");
      }
   }

   private void doBehavior()
   {
      if(!enable.get())
      {
         return;
      }

      if(stepping.poll())
      {
         if(stepping.hasChanged())
         {
            LogTools.info("Sending Steps");
         }

         if (footstepsTaken.compareAndSet(2,0))
         {
            LogTools.info("Sending Steps");

            FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
            FootstepDataListMessage foorStepList = createTwoStepInPlaceSteps(fullHumanoidRobotModel);
            behaviorHelper.publishFootstepList(foorStepList);

         }

      }

      else if (stepping.hasChanged())
      {
         LogTools.info("Stopped Stepping");
      }

      if(goToWalk.poll())
      {
//         thread.start();
         LogTools.info("Walking few steps");
         FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
         FootstepDataListMessage footstepDataListMessage = gotoWalk(fullHumanoidRobotModel);
         behaviorHelper.publishFootstepList(footstepDataListMessage);
//         System.out.println(behaviorHelper.getLatestControllerState());
      }

      if(enable.get())
      {
         if(flag1)
         {
//            System.out.println(longTime);
            FullHumanoidRobotModel fullHumanoidRobotModel = behaviorHelper.pollFullRobotModel();
//            FootstepDataListMessage footstepDataListMessage = nowTurn(fullHumanoidRobotModel);
            nowTurn();
//            behaviorHelper.publishFootstepList(nowTurn());
         }
      }
   }

//   public FootstepDataListMessage nowTurn(FullHumanoidRobotModel fullHumanoidRobotModel)
   public void nowTurn()
   {
//      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
//      for (RobotSide side : RobotSide.values())
//      {
//         MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
//         FramePoint3D footLocation = new FramePoint3D(stepFrame);
//         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame,90.0,0.0,0.0);
//         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
//         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());
//
//         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
//         footstelpList.getFootstepDataList().add().set(footstepDataMessage);
//
//      }
//      return footstelpList;
      double trajectoryTime = 1.0;
      behaviorHelper.requestChestGoHome(trajectoryTime);
      behaviorHelper.requestPelvisGoHome(trajectoryTime);

      double[] jointAngles = new double[] {0.0, -1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      behaviorHelper.requestArmTrajectory(RobotSide.LEFT, trajectoryTime, jointAngles);

      jointAngles = new double[] {0.0, 1.4, 0.0, 0.0, 0.0, 0.0, 0.0};
      behaviorHelper.requestArmTrajectory(RobotSide.RIGHT, trajectoryTime, jointAngles);
   }


   public FootstepDataListMessage gotoWalk(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      double x = 0.0;
      double y = 0.11;
      double z = 0.0;

      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
      for(int i = 0; i < 5; ++i)
      {
         for (RobotSide side : RobotSide.values())
         {
            MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
            FramePoint3D footLocation = new FramePoint3D(stepFrame,x, side.negateIfRightSide(y),z);
            FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
            footLocation.changeFrame(ReferenceFrame.getWorldFrame());
            footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

            FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
            footstelpList.getFootstepDataList().add().set(footstepDataMessage);
            x = x + 0.4;
         }
      }
      footstelpList.setAreFootstepsAdjustable(true);
      return footstelpList;
   }

   private FootstepDataListMessage createTwoStepInPlaceSteps(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      FootstepDataListMessage footstelpList = new FootstepDataListMessage();
      RecyclingArrayList<FootstepDataMessage> footstepDataMessages = footstelpList.getFootstepDataList();

      double x = 0;
      double y = -0.10;
      double z = 0;
      for(RobotSide side : RobotSide.values())
      {
//         RobotSide tmpside = RobotSide.LEFT;
         MovingReferenceFrame stepFrame = fullHumanoidRobotModel.getSoleFrame(side);
         FramePoint3D footLocation = new FramePoint3D(stepFrame);
         FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepDataMessage = HumanoidMessageTools.createFootstepDataMessage(side,footLocation,footOrientation);
         footstepDataMessages.add().set(footstepDataMessage);
         x = x + 0.4;
      }
      footstelpList.setAreFootstepsAdjustable(true);
      return footstelpList;
   }

//   public HighLevelControllerName latestControllerState()
//   {
//      return HighLevelControllerName.fromByte(controllerState.getLatest().getEndHighLevelControllerName());
//   }


   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("SearchAndKickBehavior");
      private static final CategoryTheme SearchAndKick = apiFactory.createCategoryTheme("KickThings");
      private static final Category SearchAndKickCategory = RootCategory.child(SearchAndKick);

      public static final Topic<Boolean> Stepping =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Stepping"));
      public static final Topic<Boolean> Enable =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Enable"));
      public static final Topic<Boolean> Abort =  SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("Abort"));
      public static final Topic<Boolean> Walk = SearchAndKickCategory.topic(apiFactory.createTypedTopicTheme("walking12345"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

   public static void main(String[] args)
   {
//      BehaviorHelper tmp = new BehaviorHelper();
//      new SuppaKickBehavior();
   }
}
