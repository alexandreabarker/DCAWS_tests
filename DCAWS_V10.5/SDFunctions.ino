void setupSD()
{
  //Initialize the SD Card Slot
  radio.println(F("Initializing SD card..."));
  pinMode(chipSelect, OUTPUT); //For the SD Card to work
  if (!SD.begin(chipSelect))
  {
    radio.println(F("initialization failed!"));
    return;
  }
  dcawsLog = SD.open("dcawsLog.csv", FILE_WRITE);
}

void checkSD()
{
  if (!SD.begin(chipSelect))
  {
    radio.println(F("SD card initialization failed!"));
    missionReady = false;
    return;
  }
  radio.print(F("SD card initialization confirmed and found files: "));
  if (dcawsLog)
    radio.print(F("dcawsLog, "));
}


void logData()
{
  if(dcawsLog)
  {
    //File dcawsLog = SD.open("dcawsLog.csv", FILE_WRITE);  
    dcawsLog.println(dataString);
    dcawsLog.flush();  
  }
}
