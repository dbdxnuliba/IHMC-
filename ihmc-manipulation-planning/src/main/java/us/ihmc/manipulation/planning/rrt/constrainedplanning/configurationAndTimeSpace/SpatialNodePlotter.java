package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class SpatialNodePlotter
{
   private ExploringDefinition spatialDefinition;

   private int dimensionOfConfigurations;

   private List<Plotter> plotters = new ArrayList<Plotter>();

   private int cnt;

   private boolean isFrameEnabled;

   public SpatialNodePlotter(ExploringDefinition spatialDefinition, boolean enabled)
   {
      this.spatialDefinition = spatialDefinition;
      SpatialData randomSpatialData = spatialDefinition.getRandomSpatialData();
      dimensionOfConfigurations = randomSpatialData.getExploringDimension();

      List<String> plotterNames = randomSpatialData.getExploringConfigurationNames();

      PrintTools.info("########### "+dimensionOfConfigurations +" "+ plotterNames.size());
      
      for (int i = 0; i < plotterNames.size(); i++)
      {
         Plotter plotter = new Plotter();

         plotter.setPreferredSize(400, 600);

         plotter.setViewRange(2.0);
         plotter.setXYZoomEnabled(true);
         plotter.setShowLabels(true);
         plotter.setFocusPointX(0.5);
         plotter.setFocusPointY(0.0);
         plotters.add(plotter);

         String plotterName = plotterNames.get(i);
         PrintTools.info(""+i+" "+plotterName);
         isFrameEnabled = enabled;
         if (enabled)
            plotter.showInNewWindow(plotterName, false);
      }
      cnt = 0;
   }

   public void closeAll()
   {
      for (int i = 0; i < plotters.size(); i++)
      {
         // TODO : close panels.
         if (isFrameEnabled)
            plotters.get(i).getJFrame().dispose();
      }
   }

   /**
    * 1::node
    * 2::path
    * 3::shortcut
    */
   public void update(SpatialNode node, int type)
   {
      double progress = spatialDefinition.getExploringProgress(node);
      Color color;
      double diameter = 0.01;
      
      TDoubleArrayList configurations = node.getSpatialData().getExploringConfigurations();

      for (int nodeIndex = 0; nodeIndex < dimensionOfConfigurations; nodeIndex++)
      {
         String prefix;
         switch (type)
         {
         case 1:
            if (node.isValid())
            {
               prefix = "" + cnt + "_valid_" + nodeIndex;
               color = Color.blue;
            }
            else
            {
               prefix = "" + cnt + "_invalid_" + nodeIndex;
               diameter = 0.01;
               color = Color.red;
            }
            break;
         case 2:
            prefix = "" + cnt + "_path_" + nodeIndex;
            color = Color.black;
            break;
         case 3:
            prefix = "" + cnt + "_shortcut_" + nodeIndex;
            color = Color.green;
            break;
         default:
            prefix = "";
            color = Color.white;
            break;
         }

         double configurationData = configurations.get(nodeIndex);

         if (node.getParent() != null && node.isValid())
         {
            SpatialNode parentNode = node.getParent();
            double parentProgress = spatialDefinition.getExploringProgress(parentNode);
            double parentConfigurationData = parentNode.getSpatialData().getExploringConfigurations().get(nodeIndex);

            LineArtifact lineArtifact = new LineArtifact(prefix + "_line", new Point2D(parentProgress, parentConfigurationData),
                                                         new Point2D(progress, configurationData));

            lineArtifact.setColor(color);
            plotters.get(nodeIndex).addArtifact(lineArtifact);
         }

         CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_node", progress, configurationData, diameter, true);
         nodeArtifact.setColor(color);

         plotters.get(nodeIndex).addArtifact(nodeArtifact);
         plotters.get(nodeIndex).update();
      }

      cnt++;
   }
}
