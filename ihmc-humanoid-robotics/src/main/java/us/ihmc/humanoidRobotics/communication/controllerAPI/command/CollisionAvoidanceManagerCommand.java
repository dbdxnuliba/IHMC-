package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import controller_msgs.msg.dds.CollisionAvoidanceManagerMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;

public class CollisionAvoidanceManagerCommand implements Command<CollisionAvoidanceManagerCommand, CollisionAvoidanceManagerMessage>
{

   private final PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();
   private final RecyclingArrayList<PlanarRegionCommand> planarRegions = new RecyclingArrayList<>(100, PlanarRegionCommand.class);

   @Override
   public void set(CollisionAvoidanceManagerCommand other)
   {
      clear();

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

   @Override
   public void clear()
   {
      planarRegions.clear();
   }

   @Override
   public void setFromMessage(CollisionAvoidanceManagerMessage message)
   {
      clear();

      List<PlanarRegionMessage> planarRegionsList = message.getPlanarRegionsList();
      
      for (int i = 0; i < planarRegionsList.size(); ++i) 
      {
         planarRegionCommand.setFromMessage(planarRegionsList.get(i));
         planarRegions.add().set(planarRegionCommand);
      }

   }

   @Override
   public Class<CollisionAvoidanceManagerMessage> getMessageClass()
   {
      return CollisionAvoidanceManagerMessage.class;
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
