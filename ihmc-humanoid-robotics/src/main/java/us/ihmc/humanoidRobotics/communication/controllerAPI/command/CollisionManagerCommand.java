package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import controller_msgs.msg.dds.CollisionManagerMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

public class CollisionManagerCommand implements Command<CollisionManagerCommand, CollisionManagerMessage>
{

   private float test;
   private final PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();
   private final RecyclingArrayList<PlanarRegionCommand> planarRegions = new RecyclingArrayList<>(100, PlanarRegionCommand.class);

   @Override
   public void set(CollisionManagerCommand other)
   {
      clear();

      this.test = other.test;
      RecyclingArrayList<PlanarRegionCommand> dataList = other.getPlanarRegions();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            planarRegions.add().set(dataList.get(i));
      }
   }

   public int getNumberOfPlanarRegions()
   {
      return planarRegions.size();
   }

   public RecyclingArrayList<PlanarRegionCommand> getPlanarRegions()
   {
      return planarRegions;
   }

   public PlanarRegionCommand getPlanarRegionCommand(int i)
   {
      return planarRegions.get(i);
   }

   public float getTest()
   {
      return test;
   }

   @Override
   public void clear()
   {
      test = (float) 0.0;
      planarRegions.clear();
   }

   @Override
   public void setFromMessage(CollisionManagerMessage message)
   {
      clear();

      test = message.getTest();

      List<PlanarRegionMessage> planarRegionsList = message.getPlanarRegionsList();
      
      for (int i = 0; i < planarRegionsList.size(); ++i) 
      {
         planarRegionCommand.setFromMessage(planarRegionsList.get(i));
         planarRegions.add().set(planarRegionCommand);
      }

   }

   @Override
   public Class<CollisionManagerMessage> getMessageClass()
   {
      return CollisionManagerMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return 0;
   }

}
