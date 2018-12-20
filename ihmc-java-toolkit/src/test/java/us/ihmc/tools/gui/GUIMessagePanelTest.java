package us.ihmc.tools.gui;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
@Tag("ui")
public class GUIMessagePanelTest
{
	@Test// timeout = 30000
   public void testGetText()
   {
      GUIMessagePanel guiMessagePanel = new GUIMessagePanel("Test");
      
      guiMessagePanel.appendMessage("message0");
      guiMessagePanel.appendMessage("message1");
      
      String text = guiMessagePanel.getText();
            
      assertEquals("message1\nmessage0\n", text);
   }
}
