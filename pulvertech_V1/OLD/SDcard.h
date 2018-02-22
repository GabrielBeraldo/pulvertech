#include "SD.h"
#define FilaName "Config.pv1"

bool ReadSdInfo(float *v)
{

    File config;

    Serial.println("Initializing SD card...");

    if (!SD.begin(4))
    {
        Serial.println("SD initialization failed");
        return false;
    }

    config = SD.open(FilaName, FILE_READ);
    if(!config){ 
    	Serial.println("Error opening Config.pv1");
    	return false;
    }

    int i = 0;
    while (config.available())
    {
        String buffer = config.readStringUntil(':');
        buffer = config.readStringUntil('\n');
        v[i++] = buffer.toFloat();
    }
    config.close();

    return true;
}
