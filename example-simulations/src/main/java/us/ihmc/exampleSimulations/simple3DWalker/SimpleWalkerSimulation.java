package us.ihmc.exampleSimulations.simple3DWalker;


import us.ihmc.exampleSimulations.exampleContact.PushStickRobot;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;

import us.ihmc.simulationconstructionset.ContactingExternalForcePoint;
import us.ihmc.simulationconstructionset.ContactingExternalForcePointsVisualizer;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import javax.swing.*;
import java.util.ArrayList;

public class SimpleWalkerSimulation
{
   private SimulationConstructionSet scs;
   private boolean withFeet = true;
   private boolean withInertiaControl = false;
   private boolean withImpactControl = false;
   private boolean withTwan = true;

   private boolean withPush = true;
   private double PUSH_FORCE_X = 170;
   private double PUSH_DURATION = 0.05;

   private boolean withHeightOnly = false;



   SimpleWalkerSimulation() throws SimulationExceededMaximumTimeException
   {
      double simulationDT = 1e-4;
      
      SimpleWalkerRobot robot = new SimpleWalkerRobot(withFeet, false);
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();


      SimpleWalkerController walkerController = new SimpleWalkerController(robot, simulationDT, withInertiaControl, withImpactControl, withTwan, withHeightOnly);
      walkerController.setPushForceX(PUSH_FORCE_X);
      robot.addYoGraphicsListRegistry(walkerController.getYoGraphicsListRegistry());
      robot.setController(walkerController);
    


      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      final SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();


      scs = new SimulationConstructionSet(robot);
      scs.setDT(simulationDT, 10);
      scs.setMaxBufferSize(32000);
      scs.startOnAThread();

      simulationOverheadPlotter.setDrawHistory(true);

      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getDesiredCoPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentCoPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentCoMGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getDesiredICPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentICPGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentLFootGraphicArtifact());
      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getCurrentRFootGraphicArtifact());

      simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getPushForceGraphicArtifact());

      //simulationOverheadPlotter.getPlotter().addArtifact(walkerController.getFootArtifact());

      simulationOverheadPlotter.setXVariableToTrack((YoDouble) robot.getVariable("q_x"));
      simulationOverheadPlotter.setYVariableToTrack((YoDouble) robot.getVariable("q_y"));


      ContactingExternalForcePoint contactingExternalForcePoint = walkerController.getExternalForcePoint();



      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, "Plotter", true);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend", false);

      yoGraphicsListRegistry.update();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());

      scs.startOnAThread();

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs,100);


      if (withPush)
      {
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(6.6);
         robot.getRootJoints().get(0).addExternalForcePoint(contactingExternalForcePoint);

         walkerController.setDuringPush(true);
        // walkerController.getPushForceGraphic().set(robot.getBodyPositionX(),robot.getBodyPositionY(),robot.getBodyHeight(),robot.getBodyPositionX()-0.3,robot.getBodyPositionY(),robot.getBodyHeight());
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(PUSH_DURATION);
         //walkerController.getPushForceGraphic().set(0,0,0,0,0,0);
         walkerController.setDuringPush(false);
         robot.getRootJoints().get(0).removeExternalForcePoint(contactingExternalForcePoint);
         blockingSimulationRunner.simulateAndBlockAndCatchExceptions(3.0);
      }






   }
   
   public static void main(String[] args) throws SimulationExceededMaximumTimeException
   {
     new SimpleWalkerSimulation();
   }
}
