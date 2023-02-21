# GT911
Modified GT911 library for ESP32
Based on https://github.com/blackketter/GT911


See the README.MD on https://github.com/DeanK2022/BladeOMatic for the board used and special faffing needed to get it working.

The screen is this one: https://www.aliexpress.com/item/1005004083478309.html?spm=a2g0o.order_list.order_list_main.17.21ef1802IGJi0I and has an ID on it JC3248S035R which is almost unsearchable for on the internet for example code.

I could not get the interrupt driven interface working and the vendor advised to use polling only.  Why this is I do not know, all I know is that if I tried to enable it the ESP32 would just keep rebooting when interupts were enabled.

The i2c address selection was a bit random so I added some "find i2c address" code which will fail if you have another i2c device connected to the same channel that is lower address than the one on GT911.

There was some strange behaviour when polling the coordinates register GT911_READ_COORD_ADDR  0x814E.

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


