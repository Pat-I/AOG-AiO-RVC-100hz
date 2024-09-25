#ifndef _ETHER_h
#define _ETHER_h

#include <stdint.h>
#include "elapsedMillis.h"
#include "IPAddress.h"
#include "Arduino.h"
//#include <NativeEthernet.h>
//#include <NativeEthernetUdp.h>
#include "AsyncUDP_Teensy41.h"
#include <EEPROM.h>

AsyncUDP GNSS;    // UDP object for incoming NMEA
AsyncUDP RTCM;    // UDP object for incoming RTCM
AsyncUDP PGN;     // UDP object for PGNs on port 8888
AsyncUDP PGN_OGX; // UDP object for PGNs on port 7777 from OGX


class Eth_UDP
{
public:
	IPAddress myIP = { 192, 168, 5, 126 };  // 126 default IP for steer module
  IPAddress myNetmask = {255, 255, 255, 0};
  IPAddress myGW = {192, 168, 5, 1};
  IPAddress mydnsServer = {192, 168, 5, 1};
  IPAddress broadcastIP;
  byte mac[6] = { 0x0A, 0x0F, myIP[0], myIP[1], myIP[2], myIP[3] };     // create unique MAC from IP as IP should already be unique
  IPAddress remoteIP;


	// This modules listens to GPS sent on (carry over from Ace)
  // likely not needed but may be convenient for simulating a GPS receiver on the bench using UDP
	unsigned int portGNSS_2211 = 2211;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.

	unsigned int portRTCM_2233 = 2233;     // Why 2211? 22XX=GPS then 2211=GPS1 2222=GPS2 2233=RTCM3 corrections easy to remember.

	unsigned int portSteer_8888 = 8888;    // UDP port that Modules (like this one) listen to

	unsigned int portAgIO_9999 = 9999;     // UDP port that AgIO listens to, send data here

	bool isRunning = false;                // set true with successful Eth Start()
  int8_t linkStatus = -1;                // 0 - Unknown, 1 - LinkON, 2 - LinkOFF
	const int EE_ver = 2402;               // if value in eeprom does not match, overwrite with defaults

  elapsedMillis initTimer = 2000;

  void Eth_EEPROM() {                        
    
    Serial.println();
    Serial.println("EEPROM IP Address reading");
    uint16_t eth_ee_read;
    EEPROM.get(60, eth_ee_read);

    if (eth_ee_read != EE_ver) {     // if EE is out of sync, write defaults to EE
      EEPROM.put(60, EE_ver);
      SaveModuleIP();
      Serial.print("\r\n\nWriting Eth defaults to EEPROM\r\n");
    } else {
      EEPROM.get(62, myIP[0]);
      EEPROM.get(63, myIP[1]);
      EEPROM.get(64, myIP[2]);
      mac[2] = myIP[0];
      mac[3] = myIP[1];
      mac[4] = myIP[2];
      Serial.println("EEPROM IP Address reading step 1");
    }

    broadcastIP[0] = myIP[0];
    broadcastIP[1] = myIP[1];
    broadcastIP[2] = myIP[2];
    broadcastIP[3] = 255;                // same subnet as module's IP but use broadcast
    Serial.println("EEPROM IP Address reading Step 2");
    Serial.println(broadcastIP);
    Serial.print("mac: ");
    for (int i = 0; i < sizeof(mac); i++)
    {
      printHex(mac[i]);
    }
    Serial.println();
  }

  bool init()
  {
    Serial.println("\r\nEth_UDP init");
    if (isRunning) return true;
    //Ethernet.MACAddress(mac);                 // get Teensy's internal MAC, doesn't work reliably
    //Ethernet.begin(mac, 2000, 2000);          // start dhcp connection with 2s timeout, that's enough time to get an eth linkStatus update
    //Ethernet.begin(mac, myIP);                // blocks if unplugged
    Ethernet.setDHCPEnabled(false);             // Must be set to false if using non-blocking begin() or DHCP client will wipe out static settings in 6 minutes killing the ethernet connection.
    Ethernet.begin(mac, 0);                     // non-blocking method, set IP later

    // Check for Ethernet hardware present, always returns "EthernetW5500" (3) for Teensy 4.1 w/Eth
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("\r\n\n*** Ethernet was not found. GPS via USB only ***");   // maybe using non Ethernet Teensy?
      return false;
    }

    Ethernet.setLocalIP(myIP);                  // also non-blocking as opposed to Ethernet.begin(mac, myIP) which block with unplugged/unconnected cable
    Ethernet.setSubnetMask(myNetmask);          //QNEthernet requires this to be set
    Ethernet.setGatewayIP(myGW);                //QNEthernet requires this to be set
    Ethernet.setDNSServerIP(mydnsServer);       //QNEthernet requires this to be set
    Serial.print("\r\n\nEthernet connection set with static IP address");

    Serial.print("\r\n- Using MAC address: ");
    for (byte octet = 0; octet < 6; octet++) {
      if (mac[octet] < 0x10) Serial.print("0");
      Serial.print(mac[octet], HEX);
      if (octet < 5) Serial.print(':');
    }
    /*Serial.print("\r\n- Using IP address: ");
    for (byte octet = 0; octet < 4; octet++) {
      Serial.print(myIP[octet]);
      if (octet < 3) Serial.print('.');
    }*/

    Serial.print("\r\n- Ethernet IP of module: ");
    Serial.print(Ethernet.localIP());

    Serial.print("\r\n- Ethernet Broadcast IP: ");
    Serial.print(broadcastIP);

    Serial.print("\r\n\nSetting up UDP comms");
    Serial.print("\r\n- Sending to AgIO port: ");
    Serial.print(portAgIO_9999);

    if (GNSS.listen(portGNSS_2211))
    {
      Serial.print("\r\nGNSS UDP Listening on: ");
      Serial.print(Ethernet.localIP());
      Serial.print(":");
      Serial.print(portGNSS_2211);

      // this function is triggered asynchronously(?) by the AsyncUDP library
      // GNSS.onPacket([&](AsyncUDPPacket packet)
      //               { gNSS(packet); }); // all the brackets and ending ; are necessary!
    }

    if (RTCM.listen(portRTCM_2233))
    {
      Serial.print("\r\nRTCM UDP Listening on: ");
      Serial.print(Ethernet.localIP());
      Serial.print(":");
      Serial.print(portRTCM_2233);
      // this function is triggered asynchronously(?) by the AsyncUDP library
      RTCM.onPacket([&](AsyncUDPPacket packet)
                    { nTrip(packet); }); // all the brackets and ending ; are necessary!
    }

    if (PGN.listen(portSteer_8888))
    {
      Serial.print("\r\nRTCM UDP Listening on: ");
      Serial.print(Ethernet.localIP());
      Serial.print(":");
      Serial.print(portSteer_8888);

      // this function is triggered asynchronously(?) by the AsyncUDP library
      PGN.onPacket([&](AsyncUDPPacket packet)
                   { pGN(packet); }); // all the brackets and ending ; are necessary!
    }

    isRunning = true;
    return true;
  }

  void pGN(AsyncUDPPacket packet)
  {
    if (packet.remotePort() != 9999 || packet.length() < 5) return; // make sure from AgIO
    struct pgnData udpPGN;
    memset( &udpPGN, 0, sizeof(pgnData));
    udpPGN.remoteIP = packet.remoteIP();
    udpPGN.length = packet.length();

    for (int i = 0; i < packet.length(); i++) {
      udpPGN.data[i] = packet.data()[i];
      //Serial.print("PGN_Data: ");
      //Serial.println(udpPGN.data[i], HEX);
    } 
    if (PGN_buf->add(PGN_buf,&udpPGN) == -1) Serial.println("PGN ring buffer is full!");
  }

  void nTrip(AsyncUDPPacket packet)
  {
    if (packet.remotePort() != 9999 || packet.length() < 5) return; // make sure from AgIO
    struct ntripData udpNtrip;
    memset ( &udpNtrip, 0, sizeof(ntripData));
    udpNtrip.length = packet.length();
    for (int i =0; i < packet.length(); i++){
      udpNtrip.data[i] = packet.data()[i];
    }

    if (NTRIP_buf->add(NTRIP_buf,&udpNtrip) == -1) Serial.println("NTRIP ring buffer is full!");
  }

  void gNSS(AsyncUDPPacket packet)
  {
    if (packet.remotePort() != 9999 || packet.length() < 5) return; // make sure from AgIO
    // if ( gnssRingBuffer.isFull() ) {
    //   Serial.println("PGN Ring Buffer Full"); return;
    // }
    // uint16_t size = packet.length();
    // for (int i = 4; i < size; i++) {
    //   gnssRingBuffer.push(packet.data()[i]);
    // }
    // gnssRingBuffer.push(0xFF);
  }

  void SendUdpByte(uint8_t* _data, uint8_t _length, IPAddress _ip, uint16_t _port) {
    // PGN.beginPacket(_ip, _port);
    // PGN.write(_data, _length);
    // PGN.endPacket();
    PGN.writeTo(_data, _length, _ip, _port);
  }

  void SendUdpChar(char* _charBuf, uint8_t _length, IPAddress _ip, uint16_t _port) {

    uint8_t tmpBuf[_length];
    uint8_t *tmpBufptr = tmpBuf;
    for (int i = 0; i < _length; i++)
    {
      tmpBuf[i] = _charBuf[i];
    }
    PGN.writeTo(tmpBufptr, _length, _ip, _port);
  }

  void SendUdpAry(char _data[], uint8_t _length, IPAddress _ip, uint16_t _port) {

    uint8_t tmpBuf[_length];
    uint8_t *tmpBufptr = tmpBuf;
    for (int i = 0; i < _length; i++)
    {
      tmpBuf[i] = _data[i];
    }
    PGN.writeTo(tmpBufptr, _length, _ip, _port);
  } 

  // "raw" method, bypasses limit checks in firmware but AOG should still have limits
  //uint8_t PGN_99[] = { 0x80, 0x81, 126, 0x99, n/u, 1-2, 1-10s, 'H', 'e', 'l', 'l', 'o', ' ', 'A', 'o', 'G', '!', '!' }; //, 0xCC };
  //UDP.SendUdpByte(PGN_99, sizeof(PGN_99), UDP.broadcastIP, UDP.portAgIO_9999);

  // "proper" function
  //char msg[] = "AutoSteer Switch ON";
  //char msgTime = 2;
  //UDP.SendUdpFreeForm(1, msg, strlen(msg), msgTime, UDP.broadcastIP, UDP.portAgIO_9999);  // timed popup

  //char msg[] = "Work switch";
  //UDP.SendUdpFreeForm(2, msg, strlen(msg), 1, UDP.broadcastIP, UDP.portAgIO_9999);        // interactive "OK" popup

  void SendUdpFreeForm(uint8_t _type, char _msg[], uint8_t _len, char _seconds, IPAddress dip, uint16_t dport)
  {
    char header[7] = { 0x80, 0x81, 126, 0x99, 0, 1, 1};   // free form msg PGN header, 126 is steer module ID (not used yet)
    header[5] = _type;

    _seconds = max(1, _seconds);      // limit 1-10 sec msg box time
    _seconds = min(10, _seconds);
    header[6] = _seconds;

    uint8_t ForTheWire[_len + 8];

    for (byte i = 0; i < 7; i++) {
      ForTheWire[i] = header[i];
    }

    for (byte i = 0; i < _len; i++) {
      ForTheWire[i + 7] = _msg[i];
    }

    ForTheWire[_len + 7] = 0;

    if (_type == 1) { Serial.print("\r\n"); Serial.print(ForTheWire[6], DEC); Serial.print("s timed "); }
    else Serial.print("\r\nOK ");

    Serial.print(" pop-up msg sent to AOG: "); Serial.print("\"");
    for (byte i = 7; i < sizeof(ForTheWire) - 1; i++) {
      Serial.print(ForTheWire[i]);
    }
    Serial.print("\"");

    PGN.writeTo(ForTheWire, sizeof(ForTheWire), dip, dport);

  }

  void SaveModuleIP(void) {
    //ID stored in 60
    EEPROM.put(62, myIP[0]);
    EEPROM.put(63, myIP[1]);
    EEPROM.put(64, myIP[2]);
  }

  void printHex(uint8_t num)
  {
    char hexCar[2];

    sprintf(hexCar, "%02X", num);
    Serial.print(hexCar);
  }

};
#endif
