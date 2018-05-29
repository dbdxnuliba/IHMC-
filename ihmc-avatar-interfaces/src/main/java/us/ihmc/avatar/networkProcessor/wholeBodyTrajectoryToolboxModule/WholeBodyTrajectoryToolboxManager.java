package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import us.ihmc.commons.Conversions;
import us.ihmc.manipulation.planning.exploringSpatial.ExploringDefinition;

public abstract class WholeBodyTrajectoryToolboxManager implements WholeBodyTrajectoryToolboxManagerInterface
{
   private long startTime;
   private double computationTime = 0.0;

   private int numberOfUpdate;
   private int maximumNumberOfUpdate;

   protected ExploringDefinition exploringDefinition;

   public WholeBodyTrajectoryToolboxManager(ExploringDefinition exploringDefinition, int maximumNumberOfUpdate)
   {
      this.startTime = System.nanoTime();
      this.numberOfUpdate = 0;
      this.maximumNumberOfUpdate = maximumNumberOfUpdate;
      this.exploringDefinition = exploringDefinition;
   }

   public void initialize()
   {
      this.startTime = System.nanoTime();
      this.numberOfUpdate = 0;
   }

   public void update()
   {
      numberOfUpdate++;
   }

   public void terminalManager()
   {
      long endTime = System.nanoTime();
      computationTime = Conversions.nanosecondsToSeconds(endTime - startTime);
   }

   protected boolean isExceedMaximumNumberOfUpdate()
   {
      return maximumNumberOfUpdate <= numberOfUpdate;
   }

   public void setMaximumNumberOfUpdate(int value)
   {
      maximumNumberOfUpdate = value;
   }

   public double getComputingTime()
   {
      return computationTime;
   }

}
