package us.ihmc.manipulation.planning.planarRobot;

import java.awt.Color;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import gnu.trove.list.array.TDoubleArrayList;

public class PlanarRobotVisualizer
{
   private final PlanarRobot robot;
   private final double upperLimit;
   private final double lowerLimit;

   public PlanarRobotVisualizer(PlanarRobot robot, double lowerLimit, double upperLimit)
   {
      this.robot = new PlanarRobot(robot);
      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;
   }

   public void open(String title, TDoubleArrayList jointConfiguration)
   {
      robot.setJointConfiguration(jointConfiguration);

      XYSeriesCollection data = new XYSeriesCollection();

      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         XYSeries link = new XYSeries(i + "link");
         link.add(robot.getJoint(i).getJointPoint().getX(), robot.getJoint(i).getJointPoint().getY());
         link.add(robot.getJoint(i).getEndTip().getX(), robot.getJoint(i).getEndTip().getY());
         data.addSeries(link);
      }

      JFreeChart chart = ChartFactory.createXYLineChart(title, "X", "Y", data, PlotOrientation.VERTICAL, false, true, false);
      XYPlot xyPlot = chart.getXYPlot();
      xyPlot.getRangeAxis().setRange(lowerLimit, upperLimit);
      xyPlot.getDomainAxis().setRange(lowerLimit, upperLimit);

      XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         renderer.setSeriesPaint(i, Color.RED);
      }
      xyPlot.setRenderer(renderer);

      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(500, 500));

      JFrame frame;

      frame = new JFrame(this.getClass().getSimpleName());
      frame.setLocation(200, 100);

      frame.add(chartPanel);
      frame.pack();
      frame.setVisible(true);
   }
}
