package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.commons.Conversions;

public abstract class WholeBodyTrajectoryToolboxManager implements WholeBodyTrajectoryToolboxManagerInterface
{
   private long startTime;
   private double computationTime = 0.0;
   
   private int numberOfUpdate;
   private int maximumNumberOfUpdate;
   

   public WholeBodyTrajectoryToolboxManager(int maximumNumberOfUpdate)
   {
      this.startTime = System.nanoTime();
      this.numberOfUpdate = 0;
      this.maximumNumberOfUpdate = maximumNumberOfUpdate;
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
