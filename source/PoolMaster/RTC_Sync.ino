unsigned int localPort = 8888;                  // local port to listen for UDP packets
// IPAddress timeServer(193, 190, 147, 153);    // be.pool.ntp.org NTP server
IPAddress timeServer(212, 18, 3, 19);           // pool.ntp.org NTP server
int timeZOffset = 2;                            // timezone offset in hours (2 for CET)
const int NTP_PACKET_SIZE = 48;                 // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];                       // buffer to hold incoming and outgoing packets for NTP server


EthernetUDP Udp;

//update rtc clock function
void UpdateRTC()
{
  if (!DoneForTheDay)
  {
    Udp.begin(localPort);

    // send an NTP request to time server
    DEBUG_PRINT("sent an NTP packet request to time server");
    sendNTPpacket(timeServer);

    // wait for reply or timeout
    unsigned long timeout = millis();
    int packetSize = 0;
    do {
      if ((millis() - timeout) > 500UL) {
        break;
      }
      packetSize = Udp.parsePacket();
    } while (packetSize == 0);

    if ( packetSize == NTP_PACKET_SIZE ) {
      // read the packet into the buffer
      DEBUG_PRINT("NTP packet received from time server");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);

      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      const unsigned long seventyYears = 2208988800UL;
      unsigned long epoch = secsSince1900 - seventyYears;
      epoch += (timeZOffset * 3600L);

      String ts = "timestamp: " + String(epoch);
      DEBUG_PRINT(ts);

      setTime(epoch);

      char datetime[50];
      sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d",
              year(), month(), day(),
              hour(), minute(), second());

      ts = "time: " + String(datetime);
      DEBUG_PRINT(ts);
      DoneForTheDay = true;

#if defined(CONTROLLINO_MAXI)
      //Day of the month, Day of the week, Month, Year, Hour, Minute, Seconds
      Controllino_SetTimeDate((uint8_t)day(epoch), (uint8_t)weekday(epoch), (uint8_t)month(epoch), (uint8_t)year(epoch), (uint8_t)hour(epoch), (uint8_t)minute(epoch), (uint8_t)second(epoch)); // set initial values to the RTC chip. (Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
#else //Mega2560 board specifics
      rtc.adjust(DateTime(epoch));
#endif

    }
    else
    {
      DEBUG_PRINT("Timed out");
    }
  }
}


// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress & address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
