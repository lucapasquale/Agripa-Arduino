﻿// Copyright © 2016 The Things Network
// Use of this source code is governed by the MIT license that can be found in the LICENSE file.

#ifndef _THETHINGSUNO_H_
#define _THETHINGSUNO_H_

#include <Arduino.h>
#include <Stream.h>

#define DEFAULT_WAIT_TIME 120
#define DEFAULT_SF 7
#define DEFAULT_FSB 2
#define DEFAULT_RETX 5

// Set ADR off as it is currently not supported by The Things Network
// The RN2xx3 module slows down to SF12 when no datarate commands are
// sent by the network, so disabling ADR is a work-around to avoid
// all the modules slowing down to SF12
#define ADR_SUPPORTED true

#define PWRIDX_868 1
#define PWRIDX_915 1

#define debugPrintLn(...) { if (debugStream) debugStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (debugStream) debugStream->print(__VA_ARGS__); }

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

class SmartLoRa
{
  private:
    Stream* modemStream;
    Stream* debugStream;
    String model;

    String readLine(int waitTime = DEFAULT_WAIT_TIME);
    bool waitForOK(int waitTime = DEFAULT_WAIT_TIME, String okMessage = "ok");
    String readValue(String key);
    bool sendCommand(String cmd, int waitTime = DEFAULT_WAIT_TIME);
    bool sendCommand(String cmd, String value, int waitTime = DEFAULT_WAIT_TIME);
    bool sendCommand(String cmd, const byte* buf, int length, int waitTime = DEFAULT_WAIT_TIME);
    bool enableFsbChannels(int fsb);
	void disableChannels(int fromCh, int toCh);

  public:
    int downlinkPort;
    byte downlink[64];
    void init(Stream& modemStream, Stream& debugStream);
    void reset(bool adr = true, bool maxPower = true, int sf = DEFAULT_SF, int fsb = DEFAULT_FSB, int retx = DEFAULT_RETX);
    bool personalize(const byte devAddr[4], const byte nwkSKey[16], const byte appSKey[16]);
    bool join(const byte appEui[8], const byte appKey[16]);
    int sendBytes(const byte* buffer, int length, int port = 1, bool confirm = false);
    int sendString(String message, int port = 1, bool confirm = false);
    void showStatus();
};

#endif
