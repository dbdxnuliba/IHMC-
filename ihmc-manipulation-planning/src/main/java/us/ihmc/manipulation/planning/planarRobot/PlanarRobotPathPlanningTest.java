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

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class PlanarRobotPathPlanningTest
{
   private final PlanarRobot robot;
   private final Vector2D fieldRange = new Vector2D(-10.0, 10.0);

   public PlanarRobotPathPlanningTest()
   {
      robot = new PlanarRobot();
   }

   public void testBuildingRobot()
   {
      System.out.println("testBuildingRobot");

      int numberOfJoints = 9;

      System.out.println("" + Math.PI * 10 / 180);
      robot.addBaseJoint(new Point2D(), 1.0, Math.PI * 10 / 180);

      for (int i = 0; i < numberOfJoints; i++)
      {
         robot.addJoint(1.0, Math.PI * 10 / 180);
      }

      robot.updateRobot();

      System.out.println("robot dimension is " + robot.getJointDimension());
      System.out.println("end effector position is " + robot.getLastJoint().getEndTip());

      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         System.out.println(i);
         System.out.println(robot.getJoint(i).getJointPoint());
         System.out.println(robot.getJoint(i).getEndTip());
      }
   }

   public void testJointConfiguration()
   {
      System.out.println("testJointConfiguration");

      openRobotVisualizer(robot);
   }

   public static void main(String[] args)
   {
      PlanarRobotPathPlanningTest test = new PlanarRobotPathPlanningTest();
      test.testBuildingRobot();
      test.testJointConfiguration();

      System.out.println("all tests are completed");
   }

   private void openRobotVisualizer(PlanarRobot robot)
   {
      XYSeriesCollection data = new XYSeriesCollection();

      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         XYSeries link = new XYSeries(i + "link");
         link.add(robot.getJoint(i).getJointPoint().getX(), robot.getJoint(i).getJointPoint().getY());
         link.add(robot.getJoint(i).getEndTip().getX(), robot.getJoint(i).getEndTip().getY());
         data.addSeries(link);
      }

      JFreeChart chart = ChartFactory.createXYLineChart("Planar Robot", "X", "Y", data, PlotOrientation.VERTICAL, false, true, false);
      XYPlot xyPlot = chart.getXYPlot();
      xyPlot.getRangeAxis().setRange(fieldRange.getX(), fieldRange.getY());
      xyPlot.getDomainAxis().setRange(fieldRange.getX(), fieldRange.getY());

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
