# GT911
Modified GT911 library for ESP32
Based on https://github.com/blackketter/GT911
See the README.MD on https://github.com/DeanK2022/BladeOMatic for the board used and special faffing needed to get it working.

I could not get the interrupt driven interface working and the vendor advised to use polling only.  Why this is I do not know, all I know is that if I tried to enable it the ESP32 would just keep rebooting when interupts were enabled.

There was some strange behaviour when polling the co ordinates register GT911_READ_COORD_ADDR  0x814E where a call to get the register values would show no touch, but there actually was.

Code like this:

uiint8_t count = gt911.touchcount()

if (touches > 0)
    {
        GTPoint* myPoints = m_touch.readPoints();
       
        for (uint8_t idx = 0; idx < touches; idx++)
        {
          // do stuff with points
        }
    }
 would fail to run the code GTPoint* myPoints = m_touch.readPoints();
 
 Change the line to 
 if (touches >= 0)
 
 and the code runs fine.  No idea why this is but I am but an ESP32 n00b.


