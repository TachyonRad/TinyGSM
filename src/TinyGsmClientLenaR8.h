/**
 * @file       TinyGsmClientLenaR8.h
 * @author     Volodymyr Shymansky, Michael Smith
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Apr 2024
 */

#ifndef SRC_TINYGSMCLIENTLENAR8_H_
#define SRC_TINYGSMCLIENTLENAR8_H_
// #pragma message("TinyGSM:  TinyGsmClientLenaR8")

// #define TINY_GSM_DEBUG Serial

#define TINY_GSM_MUX_COUNT 7
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE

#include "TinyGsmBattery.tpp"
#include "TinyGsmGPRS.tpp"
#include "TinyGsmGPS.tpp"
#include "TinyGsmGSMLocation.tpp"
#include "TinyGsmModem.tpp"
#include "TinyGsmSMS.tpp"
#include "TinyGsmSSL.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmTemperature.tpp"
#include "TinyGsmTime.tpp"

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM    = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#if defined       TINY_GSM_DEBUG
static const char GSM_CME_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CME ERROR:";
static const char GSM_CMS_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CMS ERROR:";
#endif

enum RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmLenaR8 : public TinyGsmModem<TinyGsmLenaR8>,
                      public TinyGsmGPRS<TinyGsmLenaR8>,
                      public TinyGsmTCP<TinyGsmLenaR8, TINY_GSM_MUX_COUNT>,
                      public TinyGsmSSL<TinyGsmLenaR8>,
                      public TinyGsmBattery<TinyGsmLenaR8>,
                      public TinyGsmGSMLocation<TinyGsmLenaR8>,
                      public TinyGsmGPS<TinyGsmLenaR8>,
                      public TinyGsmSMS<TinyGsmLenaR8>,
                      public TinyGsmTemperature<TinyGsmLenaR8>,
                      public TinyGsmTime<TinyGsmLenaR8> {
  friend class TinyGsmModem<TinyGsmLenaR8>;
  friend class TinyGsmGPRS<TinyGsmLenaR8>;
  friend class TinyGsmTCP<TinyGsmLenaR8, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmSSL<TinyGsmLenaR8>;
  friend class TinyGsmBattery<TinyGsmLenaR8>;
  friend class TinyGsmGSMLocation<TinyGsmLenaR8>;
  friend class TinyGsmGPS<TinyGsmLenaR8>;
  friend class TinyGsmSMS<TinyGsmLenaR8>;
  friend class TinyGsmTemperature<TinyGsmLenaR8>;
  friend class TinyGsmTime<TinyGsmLenaR8>;

  /*
   * Inner Client
   */
 public:
  class GsmClientLenaR8 : public GsmClient {
    friend class TinyGsmLenaR8;

   public:
    GsmClientLenaR8() {}

    explicit GsmClientLenaR8(TinyGsmLenaR8& modem, uint8_t mux = 0) {
      init(&modem, mux);
    }

    bool init(TinyGsmLenaR8* modem, uint8_t mux = 0) {
      this->at       = modem;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;
	  direct_link    = false;			//LENA-R8

      if (mux < TINY_GSM_MUX_COUNT) {
        this->mux = mux;
      } else {
        this->mux = (mux % TINY_GSM_MUX_COUNT);
      }
      at->sockets[this->mux] = this;

      return true;
    }

   public:
    virtual int connect(const char* host, uint16_t port, int timeout_s) {
      // stop();  // DON'T stop!
      TINY_GSM_YIELD();
      rx.clear();

      uint8_t oldMux = mux;
      sock_connected = at->modemConnect(host, port, &mux, false, timeout_s);
      if (mux != oldMux) {
        DBG("WARNING:  Mux number changed from", oldMux, "to", mux);
        at->sockets[oldMux] = NULL;
      }
      at->sockets[mux] = this;
      at->maintain();

      return sock_connected;
    }
    virtual int connect(IPAddress ip, uint16_t port, int timeout_s) {
      return connect(TinyGsmStringFromIp(ip).c_str(), port, timeout_s);
    }
    int connect(const char* host, uint16_t port) override {
      return connect(host, port, 120);
    }
    int connect(IPAddress ip, uint16_t port) override {
      return connect(ip, port, 120);
    }

    void stop(uint32_t maxWaitMs) {
      uint32_t startMillis = millis();
      dumpModemBuffer(maxWaitMs);
      // We want to use an async socket close because the syncrhonous close of
      // an open socket is INCREDIBLY SLOW and the modem can freeze up.  But we
      // only attempt the async close if we already KNOW the socket is open
      // because calling the async close on a closed socket and then attempting
      // opening a new socket causes the board to lock up for 2-3 minutes and
      // then finally return with a "new" socket that is immediately closed.
      // Attempting to close a socket that is already closed with a synchronous
      // close quickly returns an error.
      if (at->supportsAsyncSockets && sock_connected) {
        DBG("### Closing socket asynchronously!  Socket might remain open "
            "until arrival of +UUSOCL:",
            mux);
        // faster asynchronous close
        at->sendAT(GF("+USOCL="), mux, GF(",1"));
        // NOTE:  can take up to 120s to get a response
        at->waitResponse((maxWaitMs - (millis() - startMillis)));
        // We set the sock as disconnected right away because it can no longer
        // be used
        sock_connected = false;
      } else {
        // synchronous close
        at->sendAT(GF("+USOCL="), mux);
        // NOTE:  can take up to 120s to get a response
        at->waitResponse((maxWaitMs - (millis() - startMillis)));
        sock_connected = false;
      }
    }
    void stop() override {
      stop(135000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  };

  /*
   * Inner Secure Client
   */
 public:
  class GsmClientSecureR8 : public GsmClientLenaR8 {
   public:
    GsmClientSecureR8() {}

    explicit GsmClientSecureR8(TinyGsmLenaR8& modem, uint8_t mux = 0)
        : GsmClientLenaR8(modem, mux) {}

   public:
    int connect(const char* host, uint16_t port, int timeout_s) override {
      // stop();  // DON'T stop!
      TINY_GSM_YIELD();
      rx.clear();
      uint8_t oldMux = mux;
      sock_connected = at->modemConnect(host, port, &mux, true, timeout_s);
      if (mux != oldMux) {
        DBG("WARNING:  Mux number changed from", oldMux, "to", mux);
        at->sockets[oldMux] = NULL;
      }
      at->sockets[mux] = this;
      at->maintain();
      return sock_connected;
    }
    int connect(IPAddress ip, uint16_t port, int timeout_s) override {
      return connect(TinyGsmStringFromIp(ip).c_str(), port, timeout_s);
    }
    int connect(const char* host, uint16_t port) override {
      return connect(host, port, 120);
    }
    int connect(IPAddress ip, uint16_t port) override {
      return connect(ip, port, 120);
    }
  };

  /*
   * Constructor
   */
 public:
  explicit TinyGsmLenaR8(Stream& stream)
      : stream(stream),
        has2GFallback(false),
        supportsAsyncSockets(false) {
    memset(sockets, 0, sizeof(sockets));
  }
/////////////////////////////////////////////////
// LENA-R8 Stuff
////////////////////////////////////////////////
 public:
 
  bool setSimPIN (String pin) {
    if (pin.length() != 4) return 0;
    for (int i=0; i<4; i++){
      if (!isDigit(pin.charAt(i))) return 0;
    }
    sendAT(GF("+CLCK=\"SC\",1,\"") + pin + "\"");
    waitResponse();
    return 1;
  }
 
  bool enterSimPIN (String pin) {
    if (pin.length() != 4) return 0;
    for (int i=0; i<4; i++){
      if (!isDigit(pin.charAt(i))) return 0;
    }
    sendAT(GF("+CLCK=\"SC\",0,\"") + pin + "\"");
    waitResponse();
    return 1;
  }
 
//// HTTP
  void HTTPResetProfile() {
    sendAT(GF("+UHTTP=0"));			//reset http profile 0
    waitResponse();
    return;
  }
  
  void HTTPSetServerIP (String IPAddress, int port) {
    sendAT(GF("+UHTTP=0,0,\"") + IPAddress + "\"");
    waitResponse();
    sendAT(GF("+UHTTP=0,5,\"") + String(port) + "\"");
    waitResponse();
    return;
  }
  
  void HTTPSetServer (String serverName, int port) {
    sendAT(GF("+UHTTP=0,1,\"") + serverName + "\"");
    waitResponse();
    sendAT(GF("+UHTTP=0,5,\"") + String(port) + "\"");
    waitResponse();
    return;  
  }
  
  void HTTPSetUserPass (String username, String password) {
    sendAT(GF("+UHTTP=0,2,\"") + username + "\"");
    waitResponse();
    sendAT(GF("+UHTTP=0,3,\"") + password + "\"");
    waitResponse();
    sendAT(GF("+UHTTP=0,2,1"));						//set HTTP auth
    waitResponse();
    return;
  }
  
  void HTTPUseSSL (String certName) {
    sendAT(GF("+USECPRF=0,0,1"));
    waitResponse();
    sendAT(GF("+USECPRF=0,3,\"") + certName + "\"");
    waitResponse();
    sendAT(GF("+UHTTP=0,6,1,0"));
    waitResponse();
    return;
  }
  
  void HTTPUseSSL (String certName, String clientCertName, String clientKeyName, String password = "") {
    sendAT(GF("+USECPRF=0,0,1"));
    waitResponse();
    sendAT(GF("+USECPRF=0,3,\"") + certName + "\"");
    waitResponse();
    sendAT(GF("+USECPRF=0,5,\"") + clientCertName + "\"");
    waitResponse();
    sendAT(GF("+USECPRF=0,6,\"") + clientKeyName + "\"");
    waitResponse();
    if (password != "") {
      sendAT(GF("+USECPRF=0,7,\"") + password + "\"");
      waitResponse();
    }
    sendAT(GF("+UHTTP=0,6,1,0"));
    waitResponse();
    return;
  } 
  
  //Page 209 LENA-R8 AT Commands Manual
  void HTTPSetCustomReqHeader (String customHeader) {
    sendAT(GF("+UHTTP=0,9,\"") + customHeader + "\"");
    waitResponse();
    return;
  }
  
  void HTTPSetCookies (String cookie1, String cookie2 = "", String cookie3 = "", String cookie4 = "") {
    sendAT(GF("+UHTTPAC=0,0,0,\"") + cookie1 + "\"");
    waitResponse();
    if (cookie2 != "") {
      sendAT(GF("+UHTTPAC=0,0,1,\"") + cookie2 + "\"");
      waitResponse();
    }
    if (cookie3 != "") {
      sendAT(GF("+UHTTPAC=0,0,2,\"") + cookie3 + "\"");
      waitResponse();
    }
    if (cookie4 != "") {
      sendAT(GF("+UHTTPAC=0,0,3,\"") + cookie4 + "\"");
      waitResponse();
    }
    return;
  }
  
  void HTTPClearCookies() {
    sendAT(GF("+UHTTPAC=0,0,0,\"\""));
    waitResponse();
    sendAT(GF("+UHTTPAC=0,0,1,\"\""));
    waitResponse();
    sendAT(GF("+UHTTPAC=0,0,2,\"\""));
    waitResponse();
    sendAT(GF("+UHTTPAC=0,0,3,\"\""));
    waitResponse();
    return;
  }
  
  bool ReadHTTPResponse(long timeout = 10000L) {
    waitResponse(timeout, GF(GSM_NL "+UUHTTPCR:"));
    streamSkipUntil(',');
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    waitResponse();
    return responseresult == 1;
  }

  bool HTTPHead (String remotePath, String responseFileName) {
    sendAT(GF("+UHTTPC=0,0,\"") + remotePath + "\",\"" + responseFileName + "\""); 
    return ReadHTTPResponse();
  }
  
  bool HTTPGet (String remotePath, String responseFileName) {
    sendAT(GF("+UHTTPC=0,1,\"") + remotePath + "\",\"" + responseFileName + "\""); 
    return ReadHTTPResponse();
  } 
  
  bool HTTPDelete (String remotePath, String responseFileName) {
    sendAT(GF("+UHTTPC=0,2,\"") + remotePath + "\",\"" + responseFileName + "\""); 
    return ReadHTTPResponse();
  }
  
  bool HTTPPutFile (String remotePath, String responseFileName, String putFileName) {
    sendAT(GF("+UHTTPC=0,3,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + putFileName + "\""); 
    return ReadHTTPResponse();
  }
  
  //Content Type:  0=application/x-www-form-urlencoded; 1=text/plain; 2=application/octet-stream
  //3=multipart/form-data; 4=application/json; 5=application/xml; 6=user defined
  bool HTTPPutFile (String remotePath, String responseFileName, String putFileName, uint8_t contentType, String userContentType = "") {
    if (contentType == 6) {
	    sendAT(GF("+UHTTPC=0,3,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + putFileName + "\",\"" + contentType + "\",\"" + userContentType + "\""); 
	  } else {
	    sendAT(GF("+UHTTPC=0,3,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + putFileName + "\",\"" + contentType + "\""); 
	  }
    return ReadHTTPResponse();
  }
  
  //Content Type:  0=application/x-www-form-urlencoded; 1=text/plain; 2=application/octet-stream
  //3=multipart/form-data; 4=application/json; 5=application/xml; 6=user defined
  bool HTTPPostFile (String remotePath, String responseFileName, String postFileName, uint8_t contentType, String userContentType = "") {
    if (contentType == 6) {
      sendAT(GF("+UHTTPC=0,4,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + postFileName + "\",\"" + contentType + "\",\"" + userContentType + "\""); 
    } else {
      sendAT(GF("+UHTTPC=0,4,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + postFileName + "\",\"" + contentType + "\""); 
    }
    return ReadHTTPResponse();
  }
  
  //Content Type:  0=application/x-www-form-urlencoded; 1=text/plain; 2=application/octet-stream
  //3=multipart/form-data; 4=application/json; 5=application/xml; 6=user defined
  bool HTTPPostString (String remotePath, String responseFileName, String postString, uint8_t contentType, String userContentType = "") {
    if (contentType == 6) {
      sendAT(GF("+UHTTPC=0,5,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + postString + "\",\"" + contentType + "\",\"" + userContentType + "\""); 
    } else {
      sendAT(GF("+UHTTPC=0,5,\"") + remotePath + "\",\"" + responseFileName + "\",\"" + postString + "\",\"" + contentType + "\""); 
    }
    return ReadHTTPResponse();
  }
  
    
//// FTP
  void FTPSetServerIP (String IPAddress, int port) {
	sendAT(GF("+UFTP=0,\"") + IPAddress + "\"");
	waitResponse();
	sendAT(GF("+UFTP=7,\"") + String(port) + "\"");
	waitResponse();
	return;
  }
  
  void FTPSetServer (String serverName, int port) {
	sendAT(GF("+UFTP=1,\"") + serverName + "\"");
	waitResponse();
	sendAT(GF("+UFTP=7,\"") + String(port) + "\"");
	waitResponse();
	return;
  }
  
  void FTPSetUserPass (String username, String password) {
	sendAT(GF("+UFTP=2,\"") + username + "\"");
	waitResponse();
	sendAT(GF("+UFTP=3,\"") + password + "\"");
	waitResponse();
	return;
  }
  
  void FTPSetAccount (String account) {
	sendAT(GF("+UFTP=4,\"") + account + "\"");
	waitResponse();
	return;
  }
  
  void FTPSetTimeout (int inactivityTimeoutSeconds) {
	sendAT(GF("+UFTP=5,\"") + String(inactivityTimeoutSeconds) + "\"");
	waitResponse();
	return;
  }
  
  void FTPSetPassiveMode (bool passiveModeON) {
    if (passiveModeON) {
	  sendAT(GF("+UFTP=6,1"));
	} else {
      sendAT(GF("+UFTP=6,0"));
	}
	waitResponse();
	return;
  }
  
  void FTPUseSSL (String certName) {
	sendAT(GF("+USECPRF=1,0,1"));
	waitResponse();
	sendAT(GF("+USECPRF=1,3,\"") + certName + "\"");
	waitResponse();
	sendAT(GF("+UFTP=8,1,1"));
	waitResponse();
	return;
  }
  
  void FTPUseSSL (String certName, String clientCertName, String clientKeyName, String password = "") {
	sendAT(GF("+USECPRF=1,0,1"));
	waitResponse();
	sendAT(GF("+USECPRF=1,3,\"") + certName + "\"");
	waitResponse();
	sendAT(GF("+USECPRF=1,5,\"") + clientCertName + "\"");
	waitResponse();
	sendAT(GF("+USECPRF=1,6,\"") + clientKeyName + "\"");
	waitResponse();
	if (password != "") {
	  sendAT(GF("+USECPRF=1,7,\"") + password + "\"");
	  waitResponse();
	}
	sendAT(GF("+UFTP=8,1,1"));
	waitResponse();
	return;
  } 
  
  bool FTPLogout() {
    sendAT(GF("+UFTPC=0")); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPLogin() {
    sendAT(GF("+UFTPC=1")); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPDeleteFile (String remoteFileName) {
    sendAT(GF("+UFTPC=2,\"") + remoteFileName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPRenameFile (String remoteFileName, String newRemoteFileName) {
    sendAT(GF("+UFTPC=3,\"") + remoteFileName + "\",\"" + newRemoteFileName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPDownloadFile (String remoteFileName, String localFileName) {
    sendAT(GF("+UFTPC=4,\"") + remoteFileName + "\",\"" + localFileName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPUploadFile (String localFileName, String remoteFileName) {
    sendAT(GF("+UFTPC=5,\"") + localFileName + "\",\"" + remoteFileName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPChangeWorkingDir (String directoryName) {
    sendAT(GF("+UFTPC=8,\"") + directoryName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPCreateDir(String directoryName) {
    sendAT(GF("+UFTPC=10,\"") + directoryName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
  bool FTPDeleteDir (String directoryName) {
    sendAT(GF("+UFTPC=11,\"") + directoryName + "\""); 
	waitResponse(10000L, GF(GSM_NL "+UUFTPCR:"));
	streamSkipUntil(',');
	int8_t responseresult = streamGetIntBefore('\n');
	waitResponse();
	if(responseresult == 1) return 1;
	return 0;
  }
  
//// MQTT
  bool MQTTPublish(String message, String topic, int8_t QoS = 0, bool retainAD = false) {
    if (QoS < 0 || QoS > 2) {
      return false;
    }
    String hexmessage = "";
    char c[4];
    for (int i=0; i < message.length(); i++) {
      sprintf(c, "%02x", int(message.charAt(i)));
      hexmessage += c[0];
      hexmessage += c[1];
    }
    sendAT(GF("+UMQTTC=2,") + String(QoS) + "," + String(int(retainAD)) + ",1,\"" + topic + "\",\"" + hexmessage + "\"");
    waitResponse(10000L, GF("UUMQTTC:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }
  
  bool MQTTPublishFile(String fileName, String topic, int8_t QoS = 0, bool retainAD = false) {
    if (QoS < 0 || QoS > 2) {
      return false;
    }
    sendAT(GF("+UMQTTC=3,") + String(QoS) + "," + String(int(retainAD)) + ",\"" + topic + "\",\"" + fileName + "\"");
    waitResponse(10000L, GF("UUMQTTC:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }
  
  bool MQTTSubscribe(String topic, int8_t maxQoS = 0) {
    if (maxQoS < 0 || maxQoS > 2) {
      return false;
    }
    sendAT(GF("+UMQTTC=4,") + String(maxQoS) + ",\"" + topic + "\"");
	int8_t responseresult = waitResponse(10000L, GF("UUMQTTC: 4,1"));
	waitResponse();
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }
  
  bool MQTTUnsubscribe(String topic) {
    sendAT(GF("+UMQTTC=5,\"") + topic + "\"");
    waitResponse(10000L, GF("UUMQTTC:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }
  
  void MQTTSetClientID(String clientID) {
    sendAT(GF("+UMQTT=0,\"") + clientID + "\"");
    waitResponse();
  }
  
  bool MQTTKeepAlive(int keepalive, int linger = 10) {
    if (keepalive < 0 || keepalive > 65535) {
      return false;
    }
    if (linger < 0 || linger > 120) {
      return false;
    }
    sendAT(GF("+UMQTT=10,") + String(keepalive) + "," + String(linger));
    waitResponse();
    return true;
  }
  
  void MQTTPingBroker(bool pingON) {
    sendAT(GF("+UMQTTC=8,") + String(int(pingON)));
    waitResponse();
  }
  
  bool MQTTReadMsg(String &topic, String &message) {
    sendAT(GF("+UMQTTC=6,1"));
    int toplen = 0, msglen = 0;
    if (waitResponse(GF("+UMQTTC:")) != 1) {
      return false;
    }
    
    streamSkipUntil(',');//6
    streamSkipUntil(',');//qos
    streamSkipUntil(',');//top+msg length
    //streamSkipUntil(',');//top length
    toplen = streamGetIntBefore(',');
    streamSkipUntil('\"');
    topic.reserve(toplen);
    for (int i=0; i < toplen; i++) {
      while (!stream.available()){}
      topic += (char)stream.read();
    }
    streamSkipUntil(',');
    msglen = streamGetIntBefore(',');
    streamSkipUntil('\"');
    message.reserve(msglen);
    for(int i=0; i < msglen; i++){
      while(!stream.available()) {}
      message += (char)stream.read();
    }
    waitResponse();
    return true;
  }

  bool MQTTLogin() {
    sendAT(GF("+UMQTTC=1"));
    waitResponse(10000L, GF("UUMQTTC:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }
  
  bool MQTTLogout() {
    sendAT(GF("+UMQTTC=0"));
    waitResponse(5000L, GF("UUMQTTC:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    if(responseresult == 1) {
      return 1;
    }
    return 0;
  }

  bool MQTTSaveProfile() {
    sendAT(GF("+UMQTTNV=2"));
    waitResponse(GF("UMQTTNV:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    waitResponse();
    return responseresult == 1;
  }
  
  bool MQTTLoadProfile() {
    sendAT(GF("+UMQTTNV=1"));
    waitResponse(GF("UMQTTNV:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    return responseresult == 1;
  }
  
  bool MQTTResetProfile() {
    sendAT(GF("+UMQTTNV=0"));
    waitResponse(GF("UMQTTNV:"));
    streamSkipUntil(',');
    int8_t responseresult = streamGetIntBefore('\n');
    return responseresult == 1;
  }
  
  void MQTTUseSSL(String certName) {
    sendAT(GF("+USECPRF=2,0,1"));
    waitResponse();
    sendAT(GF("+USECPRF=2,3,\"") + certName + "\"");
    waitResponse();
    sendAT(GF("+UMQTT=11,1,2"));
    waitResponse();
  } 
   
  void MQTTUseSSL (String certName, String clientCertName, String clientKeyName, String password = "") {
    sendAT(GF("+USECPRF=2,0,1"));
    waitResponse();
    sendAT(GF("+USECPRF=2,3,\"") + certName + "\"");
    waitResponse();
    sendAT(GF("+USECPRF=2,5,\"") + clientCertName + "\"");
    waitResponse();
    sendAT(GF("+USECPRF=2,6,\"") + clientKeyName + "\"");
    waitResponse();
    if (password != "") {
      sendAT(GF("+USECPRF=2,7,\"") + password + "\"");
      waitResponse();
    }
    sendAT(GF("+UMQTT=11,1,2"));
    waitResponse();
  } 
   
  void MQTTSetBroker(String server, int port) {
    sendAT(GF("+UMQTT=2,\"") + server + "\"," + port);
    waitResponse();
  }
  
  void MQTTSetBrokerIP(String serverIP, int port) {
    sendAT(GF("+UMQTT=3,\"") + serverIP + "\"," + port);
    waitResponse();
  }
  
  void MQTTSetUserPass(String user, String pass) {
    sendAT(GF("+UMQTT=4,\"") + user + "\",\"" + pass + "\"");
    waitResponse();
  }
  
  bool MQTTLastWill (int QoS, bool retainWillAD, String willTopic, String willMsg) {
    if (QoS < 0 || QoS > 2) {
      return false;
    }
    sendAT(GF("+UMQTT=6,") + QoS);
    waitResponse();
    sendAT(GF("+UMQTT=7,") + int(retainWillAD));
    waitResponse();
    sendAT(GF("+UMQTT=8,\"") + willTopic + "\"");
    waitResponse();
    sendAT(GF("+UMQTT=9,\"") + willMsg + "\"");
    waitResponse();
    return true;
  }
  
  void MQTTCleanSession(bool cleanSession) {
    if (cleanSession) {
      sendAT(GF("+UMQTT=12,1"));
    } else {
      sendAT(GF("+UMQTT=12,0"));
    }
    waitResponse();
  }


////SSL Certificates
  void addRootCert(String name, const char data[]) {
    int datalength = strlen(data);
    sendAT(GF("+USECMNG=0,0,\"") + name + "\"," + datalength);
    waitResponse(">");
    for (int i = 0; i <= datalength; i++) {
      stream.write(data[i]);
    }
    waitResponse();
  }
  
  void delRootCert(String name) {
    sendAT(GF("+USECMNG=2,0,\"") + name + "\"");	
    waitResponse();
  }
  
  void addClientCert(String name, const char data[]) {
    int datalength = strlen(data);
    sendAT(GF("+USECMNG=0,1,\"") + name + "\"," + datalength);
    waitResponse(">");
    for (int i = 0; i <= datalength; i++) {
      stream.write(data[i]);
    }	
    waitResponse();
  }
  
  void delClientCert(String name) {
    sendAT(GF("+USECMNG=2,1,\"") + name + "\"");	
    waitResponse();
  }
  
  void addClientKey(String name, const char data[]) {
    int datalength = strlen(data);
    sendAT(GF("+USECMNG=0,2,\"") + name + "\"," + datalength);
    waitResponse(">");
    for (int i = 0; i <= datalength; i++) {
      stream.write(data[i]);
    }
    waitResponse();
  }
  
  void delClientKey(String name) {
    sendAT(GF("+USECMNG=2,2,\"") + name + "\"");	
    waitResponse();
  }

//// File  
  void addRootCertFile(String name, String filename) {
    sendAT(GF("+USECMNG=1,0,\"") + name + "\",\"" + filename + "\"");
    //waitResponse(10000, GF(GSM_NL "+USECMNG"));	
    waitResponse();
  }

  void addClientCertFile(String name, String filename) {
    sendAT(GF("+USECMNG=1,1,\"") + name + "\",\"" + filename + "\"");	
    waitResponse();
  }
  
  void addClientKeyFile(String name, String filename) {
    sendAT(GF("+USECMNG=1,2,\"") + name + "\",\"" + filename + "\"");
    waitResponse();
  }

  void makeFile(String filename, const char data[]){
    int datalength = strlen(data);
    sendAT(GF("+UDWNFILE=\"") + filename + "\"," + datalength);
    waitResponse(">");
    for (int i = 0; i <= datalength; i++) {
    	stream.write(data[i]);
    }
    waitResponse();
  }
  
  bool delFile(String filename){
    sendAT(GF("+UDELFILE=\"") + filename + "\"");
    if (waitResponse("OK") != 1) { 
      waitResponse();
      return false;
    }
    waitResponse();
    return true;
  }
  
  int freeFileSpace(){
    sendAT(GF("+ULSTFILE=1"));
    waitResponse("+ULSTFILE: ");
    String res = stream.readStringUntil('\n');
    char c[8];
    res.toCharArray(c, 8);
    int result = atoi(c);
    return result;
  }
  
  int getFileSize(String filename){
    sendAT("+URDFILE=\"" + filename + "\"");
    if (waitResponse(GF("+URDFILE:")) != 1) {
      return 0;
    }
    //waitResponse(GF("+URDFILE:"));
    streamSkipUntil(',');
    int result = streamGetIntBefore(',') - 1;
    return result;
  }
  
  void getFileData(String filename, char* outChar, int fileSize){
    sendAT("+URDFILE=\"" + filename + "\"");
    waitResponse(GF("+URDFILE:"));
    streamSkipUntil(',');
    streamSkipUntil('\"');
    String result = "";
    for (int i = 0; i <= fileSize; i++) {
      while(!stream.available()){}
      outChar[i] = stream.read();
    }
    return;
  }
  
 ///////////////////////////////////////////////
 //// END OF LENA-R8 STUFF
 ///////////////////////////////////////////////

  /*
   * Basic functions
   */
 protected:
  String getLocalIPImpl() {
    sendAT(GF("+CGPADDR=1"));
    if (waitResponse(GF("+CGPADDR:")) != 1) { return ""; }
    streamSkipUntil('\"');
    String res = stream.readStringUntil('\"');
    waitResponse();
    if (res.startsWith("IPV4:"))
    {
      return res.substring(5);
    }
    
    return res;

    // thisModem().streamSkipUntil(',');  // Skip context id
    // String res = thisModem().stream.readStringUntil('\r');
    // if (thisModem().waitResponse() != 1) { return ""; }
    // return res;

    // sendAT(GF("+CNACT?"));
    // if (waitResponse(GF(GSM_NL "+CNACT:")) != 1) { return ""; }
    // streamSkipUntil('\"');
    // String res = stream.readStringUntil('\"');
    // waitResponse();
    // return res;
  }

  bool initImpl(const char* pin = NULL) {
    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
    DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientLenaR8"));

    if (!testAT()) { return false; }

    sendAT(GF("E0"));  // Echo Off
    if (waitResponse() != 1) { return false; }

#ifdef TINY_GSM_DEBUG
    sendAT(GF("+CMEE=2"));  // turn on verbose error codes
#else
    sendAT(GF("+CMEE=0"));  // turn off error codes
#endif
    waitResponse();

    String modemName = getModemName();
    DBG(GF("### Modem:"), modemName);
    has2GFallback = true;
    supportsAsyncSockets = true;

    // Enable automatic time zome update
    sendAT(GF("+CTZU=1"));
    if (waitResponse(10000L) != 1) { return false; }

    SimStatus ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock the sim
    if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  // only difference in implementation is the warning on the wrong type
  String getModemNameImpl() {
    sendAT(GF("+CGMI"));
    String res1;
    if (waitResponse(1000L, res1) != 1) { return "u-blox Cellular Modem"; }
    res1.replace(GSM_NL "OK" GSM_NL, "");
    res1.trim();

    sendAT(GF("+GMM"));
    String res2;
    if (waitResponse(1000L, res2) != 1) { return "u-blox Cellular Modem"; }
    res2.replace(GSM_NL "OK" GSM_NL, "");
    res2.trim();

    String name = res1 + String(' ') + res2;
    DBG("### Modem:", name);
    // if (!name.startsWith("u-blox LENA-R8")) {
    //   DBG("### WARNING:  You are using the wrong TinyGSM modem!");
    // }

    return name;
  }

  bool factoryDefaultImpl() {
    sendAT(GF("&F"));  // Resets the current profile, other NVM not affected
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */
 protected:
  // using +CFUN=15 instead of the more common CFUN=1,1
  bool restartImpl(const char* pin = NULL) {
    if (!testAT()) { return false; }
    if (!setPhoneFunctionality(15)) { return false; }
    delay(3000);  // TODO(?):  Verify delay timing here
    return init(pin);
  }

  bool powerOffImpl() {
    sendAT(GF("+CPWROFF"));
    return waitResponse(40000L) == 1;
  }

  bool sleepEnableImpl(bool enable = true) TINY_GSM_ATTR_NOT_AVAILABLE;

  bool setPhoneFunctionalityImpl(uint8_t fun, bool reset = false) {
    sendAT(GF("+CFUN="), fun, reset ? ",1" : "");
    return waitResponse(10000L) == 1;
  }

  /*
   * Generic network functions
   */
 public:
  RegStatus getRegistrationStatus() {
    // Check first for EPS registration
    RegStatus epsStatus = (RegStatus)getRegistrationStatusXREG("CEREG");

    // If we're connected on EPS, great!
    if (epsStatus == REG_OK_HOME || epsStatus == REG_OK_ROAMING) {
      return epsStatus;
    } else {
      // Otherwise, check generic network status
      return (RegStatus)getRegistrationStatusXREG("CREG");
    }
  }

 protected:
  bool isNetworkConnectedImpl() {
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

 public:
  bool setURAT(uint8_t urat) {
    // AT+URAT=<SelectedAcT>[,<PreferredAct>[,<2ndPreferredAct>]]

    sendAT(GF("+COPS=2"));  // Deregister from network
    if (waitResponse() != 1) { return false; }
    sendAT(GF("+URAT="), urat);  // Radio Access Technology (RAT) selection
    if (waitResponse() != 1) { return false; }
    sendAT(GF("+COPS=0"));  // Auto-register to the network
    if (waitResponse() != 1) { return false; }
    return restart();
  }

  /*
   * GPRS functions
   */
 protected:
  bool gprsConnectImpl(const char* apn, const char* user = NULL,
                       const char* pwd = NULL) {
    // gprsDisconnect();

    //sendAT(GF("+CGATT=1"));  // attach to GPRS
    //if (waitResponse(360000L) != 1) { return false; }

    // Using CGDCONT sets up an "external" PCP context, i.e. a data connection
    // using the external IP stack (e.g. Windows dial up) and PPP link over the
    // serial interface.  This is the only command set supported by the LTE-M
    // and LTE NB-IoT modules

    // Set the authentication
    if (user && strlen(user) > 0) {
      sendAT(GF("+CGAUTH=1,0,\""), user, GF("\",\""), pwd, '"');
      waitResponse();
    }

    sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');  // Define PDP context 1
    waitResponse();

    sendAT(GF("+CGACT=1,1"));  // activate PDP profile/context 1
    if (waitResponse(150000L) != 1) { return false; }

    return true;
  }

  bool gprsDisconnectImpl() {
    // Mark all the sockets as closed
    // This ensures that asynchronously closed sockets are marked closed
    for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
      GsmClientLenaR8* sock = sockets[mux];
      if (sock && sock->sock_connected) { sock->sock_connected = false; }
    }

    //sendAT(GF("+CGACT=0,1"));  // Deactivate PDP context 1
    sendAT(GF("+CGACT=0"));  // Deactivate all contexts
    if (waitResponse(40000L) != 1) {
      // return false;
    }

    //sendAT(GF("+CGATT=0"));  // detach from GPRS
    //if (waitResponse(360000L) != 1) { return false; }

    return true;
  }

  /*
   * SIM card functions
   */
 protected:
  // This uses "CGSN" instead of "GSN"
  String getIMEIImpl() {
    sendAT(GF("+CGSN"));
    if (waitResponse(GF(GSM_NL)) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  /*
   * Messaging functions
   */
 protected:
  String sendUSSDImpl(const String& code) TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool   sendSMS_UTF16Impl(const String& number, const void* text,
                           size_t len) TINY_GSM_ATTR_NOT_IMPLEMENTED;

  /*
   * GSM/GPS/GNSS/GLONASS Location functions
   * NOTE:  u-blox modules use the same function to get location data from both
   * GSM tower triangulation and from dedicated GPS/GNSS/GLONASS receivers.  The
   * only difference in which sensor the data is requested from.  If a GNSS
   * location is requested from a modem without a GNSS receiver installed on the
   * I2C port, the GSM-based "Cell Locate" location will be returned instead.
   */
 protected:
  bool enableGPSImpl() {
    // AT+UGPS=<mode>[,<aid_mode>[,<GNSS_systems>]]
    // <mode> - 0: GNSS receiver powered off, 1: on
    // <aid_mode> - 0: no aiding (default)
    // <GNSS_systems> - 3: GPS + SBAS (default)
    sendAT(GF("+UGPS=1,0,3"));
    if (waitResponse(10000L, GF(GSM_NL "+UGPS:")) != 1) { return false; }
    return waitResponse(10000L) == 1;
  }
  bool disableGPSImpl() {
    sendAT(GF("+UGPS=0"));
    if (waitResponse(10000L, GF(GSM_NL "+UGPS:")) != 1) { return false; }
    return waitResponse(10000L) == 1;
  }
  String inline getUbloxLocationRaw(int8_t sensor) {
    // AT+ULOC=<mode>,<sensor>,<response_type>,<timeout>,<accuracy>
    // <mode> - 2: single shot position
    // <sensor> - 0: use the last fix in the internal database and stop the GNSS
    //          receiver
    //          - 1: use the GNSS receiver for localization
    //          - 2: use cellular CellLocate location information
    //          - 3: ?? use the combined GNSS receiver and CellLocate service
    //          information ?? - Docs show using sensor 3 and it's
    //          documented for the +UTIME command but not for +ULOC
    // <response_type> - 0: standard (single-hypothesis) response
    // <timeout> - Timeout period in seconds
    // <accuracy> - Target accuracy in meters (1 - 999999)
    sendAT(GF("+ULOC=2,"), sensor, GF(",0,120,1"));
    // wait for first "OK"
    if (waitResponse(10000L) != 1) { return ""; }
    // wait for the final result - wait full timeout time
    if (waitResponse(120000L, GF(GSM_NL "+UULOC:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }
  String getGsmLocationRawImpl() {
    return getUbloxLocationRaw(2);
  }
  String getGPSrawImpl() {
    return getUbloxLocationRaw(1);
  }

  inline bool getUbloxLocation(int8_t sensor, float* lat, float* lon,
                               float* speed = 0, float* alt = 0, int* vsat = 0,
                               int* usat = 0, float* accuracy = 0,
                               int* year = 0, int* month = 0, int* day = 0,
                               int* hour = 0, int* minute = 0,
                               int* second = 0) {
    // AT+ULOC=<mode>,<sensor>,<response_type>,<timeout>,<accuracy>
    // <mode> - 2: single shot position
    // <sensor> - 2: use cellular CellLocate location information
    //          - 0: use the last fix in the internal database and stop the GNSS
    //          receiver
    //          - 1: use the GNSS receiver for localization
    //          - 3: ?? use the combined GNSS receiver and CellLocate service
    //          information ?? - Docs show using sensor 3 and it's documented
    //          for the +UTIME command but not for +ULOC
    // <response_type> - 0: standard (single-hypothesis) response
    // <timeout> - Timeout period in seconds
    // <accuracy> - Target accuracy in meters (1 - 999999)
    sendAT(GF("+ULOC=2,"), sensor, GF(",0,120,1"));
    // wait for first "OK"
    if (waitResponse(10000L) != 1) { return false; }
    // wait for the final result - wait full timeout time
    if (waitResponse(120000L, GF(GSM_NL "+UULOC: ")) != 1) { return false; }

    // +UULOC: <date>, <time>, <lat>, <long>, <alt>, <uncertainty>, <speed>,
    // <direction>, <vertical_acc>, <sensor_used>, <SV_used>, <antenna_status>,
    // <jamming_status>

    // init variables
    float ilat         = 0;
    float ilon         = 0;
    float ispeed       = 0;
    float ialt         = 0;
    int   iusat        = 0;
    float iaccuracy    = 0;
    int   iyear        = 0;
    int   imonth       = 0;
    int   iday         = 0;
    int   ihour        = 0;
    int   imin         = 0;
    float secondWithSS = 0;

    // Date & Time
    iday         = streamGetIntBefore('/');    // Two digit day
    imonth       = streamGetIntBefore('/');    // Two digit month
    iyear        = streamGetIntBefore(',');    // Four digit year
    ihour        = streamGetIntBefore(':');    // Two digit hour
    imin         = streamGetIntBefore(':');    // Two digit minute
    secondWithSS = streamGetFloatBefore(',');  // 6 digit second with subseconds

    ilat = streamGetFloatBefore(',');  // Estimated latitude, in degrees
    ilon = streamGetFloatBefore(',');  // Estimated longitude, in degrees
    ialt = streamGetFloatBefore(
        ',');         // Estimated altitude, in meters - only forGNSS
                      // positioning, 0 in case of CellLocate
    if (ialt != 0) {  // values not returned for CellLocate
      iaccuracy =
          streamGetFloatBefore(',');       // Maximum possible error, in meters
      ispeed = streamGetFloatBefore(',');  // Speed over ground m/s3
      streamSkipUntil(',');  // Course over ground in degree (0 deg - 360 deg)
      streamSkipUntil(',');  // Vertical accuracy, in meters
      streamSkipUntil(',');  // Sensor used for the position calculation
      iusat = streamGetIntBefore(',');  // Number of satellite used
      streamSkipUntil(',');             // Antenna status
      streamSkipUntil('\n');            // Jamming status
    } else {
      iaccuracy =
          streamGetFloatBefore('\n');  // Maximum possible error, in meters
    }

    // Set pointers
    if (lat != NULL) *lat = ilat;
    if (lon != NULL) *lon = ilon;
    if (speed != NULL) *speed = ispeed;
    if (alt != NULL) *alt = ialt;
    if (vsat != NULL) *vsat = 0;  // Number of satellites viewed not reported;
    if (usat != NULL) *usat = iusat;
    if (accuracy != NULL) *accuracy = iaccuracy;
    if (iyear < 2000) iyear += 2000;
    if (year != NULL) *year = iyear;
    if (month != NULL) *month = imonth;
    if (day != NULL) *day = iday;
    if (hour != NULL) *hour = ihour;
    if (minute != NULL) *minute = imin;
    if (second != NULL) *second = static_cast<int>(secondWithSS);

    // final ok
    waitResponse();
    return true;
  }
  bool getGsmLocationImpl(float* lat, float* lon, float* accuracy = 0,
                          int* year = 0, int* month = 0, int* day = 0,
                          int* hour = 0, int* minute = 0, int* second = 0) {
    return getUbloxLocation(2, lat, lon, 0, 0, 0, 0, accuracy, year, month, day,
                            hour, minute, second);
  }
  bool getGPSImpl(float* lat, float* lon, float* speed = 0, float* alt = 0,
                  int* vsat = 0, int* usat = 0, float* accuracy = 0,
                  int* year = 0, int* month = 0, int* day = 0, int* hour = 0,
                  int* minute = 0, int* second = 0) {
    return getUbloxLocation(1, lat, lon, speed, alt, vsat, usat, accuracy, year,
                            month, day, hour, minute, second);
  }

  /*
   * Time functions
   */
 protected:
  // Can follow the standard CCLK function in the template

  /*
   * Battery functions
   */
 protected:
  uint16_t getBattVoltageImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  int8_t getBattPercentImpl() {
    sendAT(GF("+CIND?"));
    if (waitResponse(GF(GSM_NL "+CIND:")) != 1) { return 0; }

    int8_t res     = streamGetIntBefore(',');
    int8_t percent = res * 20;  // return is 0-5
    // Wait for final OK
    waitResponse();
    return percent;
  }

  uint8_t getBattChargeStateImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  bool getBattStatsImpl(uint8_t& chargeState, int8_t& percent,
                        uint16_t& milliVolts) {
    chargeState = 0;
    percent     = getBattPercent();
    milliVolts  = 0;
    return true;
  }

  /*
   * Temperature functions
   */
 protected:
  float getTemperatureImpl() {
    // First make sure the temperature is set to be in celsius
    sendAT(GF("+UTEMP=0"));  // Would use 1 for Fahrenheit
    if (waitResponse() != 1) { return static_cast<float>(-9999); }
    sendAT(GF("+UTEMP?"));
    if (waitResponse(GF(GSM_NL "+UTEMP:")) != 1) {
      return static_cast<float>(-9999);
    }
    int16_t res  = streamGetIntBefore('\n');
    float   temp = -9999;
    if (res != -1) { temp = (static_cast<float>(res)) / 10; }
    return temp;
  }

  /*
   * Client related functions
   */
 protected:
  bool modemConnect(const char* host, uint16_t port, uint8_t* mux,
                    bool ssl = false, int timeout_s = 120) {
    uint32_t timeout_ms  = ((uint32_t)timeout_s) * 1000;
    uint32_t startMillis = millis();

    // create a socket
    sendAT(GF("+USOCR=6"));
    // reply is +USOCR: ## of socket created
    if (waitResponse(GF(GSM_NL "+USOCR:")) != 1) { return false; }
    *mux = streamGetIntBefore('\n');
    waitResponse();
    if (ssl) {
      sendAT(GF("+USOSEC="), *mux, ",1");
      waitResponse();
    }

    // Enable NODELAY
    // AT+USOSO=<socket>,<level>,<opt_name>,<opt_val>[,<opt_val2>]
    // <level> - 0 for IP, 6 for TCP, 65535 for socket level options
    // <opt_name> TCP/1 = no delay (do not delay send to coalesce packets)
    // NOTE:  Enabling this may increase data plan usage
    // sendAT(GF("+USOSO="), *mux, GF(",6,1,1"));
    // waitResponse();

    // Enable KEEPALIVE, 30 sec
    // sendAT(GF("+USOSO="), *mux, GF(",6,2,30000"));
    // waitResponse();

    // connect on the allocated socket

    // Use an asynchronous open to reduce the number of terminal freeze-ups
    // This is still blocking until the URC arrives
    if (supportsAsyncSockets) {
      DBG("### Opening socket asynchronously!  Socket cannot be used until "
          "the URC '+UUSOCO' appears.");
      sendAT(GF("+USOCO="), *mux, ",\"", host, "\",", port, ",1");
      if (waitResponse(timeout_ms - (millis() - startMillis),
                       GF(GSM_NL "+UUSOCO:")) == 1) {
        streamGetIntBefore(',');  // skip repeated mux
        int8_t connection_status = streamGetIntBefore('\n');
        DBG("### Waited", millis() - startMillis, "ms for socket to open");

        //LENA-r8 start
        sendAT(GF("+USOSO="), *mux ,",65535,8,1");					//LENA-R8 keep socket alive
        waitResponse();
        sendAT(GF("+USOSO="), *mux ,",65535,128,1,3000");			//LENA-R8 idle on close if data present
        waitResponse();
        sendAT(GF("+USODL="), *mux);
        waitResponse(GF("CONNECT" GSM_NL));
        sockets[*mux]->sock_available = 0;
        sockets[*mux]->direct_link = true;
        //lena-r8 end

        return (0 == connection_status);
      } else {
        DBG("### Waited", millis() - startMillis,
            "but never got socket open notice");
        return false;
      }
    } else {
      // use synchronous open
      sendAT(GF("+USOCO="), *mux, ",\"", host, "\",", port);
      int8_t rsp = waitResponse(timeout_ms - (millis() - startMillis));
      return (1 == rsp);
    }
  }

  size_t modemSend(const void* buff, size_t len, uint8_t mux) {
	  //lena-r8 start
    if (sockets[mux]->direct_link) {
      size_t sent = stream.write(reinterpret_cast<const uint8_t*>(buff), len);
      stream.flush();
      return sent;
	  }
	  //lena-r8 end
    sendAT(GF("+USOWR="), mux, ',', (uint16_t)len);
    if (waitResponse(GF("@")) != 1) { return 0; }
    // 50ms delay, see AT manual section 25.10.4
    delay(50);
    stream.write(reinterpret_cast<const uint8_t*>(buff), len);
    stream.flush();
    if (waitResponse(GF(GSM_NL "+USOWR:")) != 1) { return 0; }
    streamSkipUntil(',');  // Skip mux
    int16_t sent = streamGetIntBefore('\n');
    waitResponse();  // sends back OK after the confirmation of number sent
    return sent;
  }

  size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux]) return 0;
    //lena-r8 start
    if (sockets[mux]->direct_link) {
      //const char _dl_dc[] = "\r\nDISCONNECT\r\n\r\nOK\r\n\r\n+UUSOCL:";
      //_dl_dc_seq = 0;
      size_t len = stream.available();
      if (len > size) {
        len = size;
      }
      for (int i = 0; i < len; i++) { 
        moveCharFromStreamToFifo(mux);
      }
      sockets[mux]->sock_available = modemGetAvailable(mux);
      return len;
    }
	  //lena-r8 end
	  sendAT();
	  waitResponse();
    sendAT(GF("+USORD="), mux, ',', (uint16_t)size);
    if (waitResponse(GF(GSM_NL "+USORD:")) != 1) { return 0; }
    streamSkipUntil(',');  // Skip mux
    int16_t len = streamGetIntBefore(',');
    streamSkipUntil('\"');

    for (int i = 0; i < len; i++) { moveCharFromStreamToFifo(mux); }
    streamSkipUntil('\"');
    waitResponse();
    // DBG("### READ:", len, "from", mux);
    sockets[mux]->sock_available = modemGetAvailable(mux);
    return len;
	
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]) return 0;
    //lena-r8 start
    if (sockets[mux]->direct_link) {
      size_t  result = 0;
      result = stream.available();
      if (!result) {
        sockets[mux]->sock_connected = modemGetConnected(mux);
      }
      return result;
    }
    // NOTE:  Querying a closed socket gives an error "operation not allowed"
    //sendAT(GF("+USORD=?"));
    //waitResponse();
    sendAT(GF("+USORD="), mux, ",0");
    size_t  result = 0;
    uint8_t res    = waitResponse(GF(GSM_NL "+USORD:"));
    // Will give error "operation not allowed" when attempting to read a socket
    // that you have already told to close
    if (res == 1) {
      streamSkipUntil(',');  // Skip mux
      result = streamGetIntBefore('\n');
      // if (result) DBG("### DATA AVAILABLE:", result, "on", mux);
      waitResponse();
    }
    if (!result) { sockets[mux]->sock_connected = modemGetConnected(mux); }
    // DBG("### Available:", result, "on", mux);
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    //lena-r8 start
    if (sockets[mux]->direct_link) {
      // AT command does not work in direct link mode, connection cannot be checked
      // TODO: How to handle disconnection in direct link mode?
      return true;
    }
    //lena-r8 end
    // NOTE:  Querying a closed socket gives an error "operation not allowed"
    sendAT(GF("+USOCTL="), mux, ",10");
    uint8_t res = waitResponse(GF(GSM_NL "+USOCTL:"));
    if (res != 1) { return false; }

    streamSkipUntil(',');  // Skip mux
    streamSkipUntil(',');  // Skip type
    int8_t result = streamGetIntBefore('\n');
    // 0: the socket is in INACTIVE status (it corresponds to CLOSED status
    // defined in RFC793 "TCP Protocol Specification" [112])
    // 1: the socket is in LISTEN status
    // 2: the socket is in SYN_SENT status
    // 3: the socket is in SYN_RCVD status
    // 4: the socket is in ESTABILISHED status
    // 5: the socket is in FIN_WAIT_1 status
    // 6: the socket is in FIN_WAIT_2 status
    // 7: the sokcet is in CLOSE_WAIT status
    // 8: the socket is in CLOSING status
    // 9: the socket is in LAST_ACK status
    // 10: the socket is in TIME_WAIT status
    waitResponse();
    return (result != 0);
  }

  /*
   * Utilities
   */
 public:
  // TODO(vshymanskyy): Optimize this!
  int8_t waitResponse(uint32_t timeout_ms, String& data,
                      GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    /*String r1s(r1); r1s.trim();
    String r2s(r2); r2s.trim();
    String r3s(r3); r3s.trim();
    String r4s(r4); r4s.trim();
    String r5s(r5); r5s.trim();
    DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
    data.reserve(64);
    uint8_t  index       = 0;
    uint32_t startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        TINY_GSM_YIELD();
        int8_t a = stream.read();
        if (a <= 0) continue;  // Skip 0x00 bytes, just in case
        data += static_cast<char>(a);
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
#if defined TINY_GSM_DEBUG
          if (r3 == GFP(GSM_CME_ERROR)) {
            streamSkipUntil('\n');  // Read out the error
          }
#endif
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith(GF("+UUSORD:"))) {
          int8_t  mux = streamGetIntBefore(',');
          int16_t len = streamGetIntBefore('\n');
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->got_data = true;
            // max size is 1024
            if (len >= 0 && len <= 1024) { sockets[mux]->sock_available = len; }
          }
          data = "";
          DBG("### URC Data Received:", len, "on", mux);
        } else if (data.endsWith(GF("+UUSOCL:"))) {
          int8_t mux = streamGetIntBefore('\n');
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->sock_connected = false;
          }
          data = "";
          DBG("### URC Sock Closed: ", mux);
        } else if (data.endsWith(GF("+UUSOCO:"))) {
          int8_t mux          = streamGetIntBefore('\n');
          int8_t socket_error = streamGetIntBefore('\n');
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux] &&
              socket_error == 0) {
            sockets[mux]->sock_connected = true;
          }
          data = "";
          DBG("### URC Sock Opened: ", mux);
        }
      }
    } while (millis() - startMillis < timeout_ms);
  finish:
    if (!index) {
      data.trim();
      if (data.length()) { DBG("### Unhandled:", data); }
      data = "";
    }
    // data.replace(GSM_NL, "/");
    // DBG('<', index, '>', data);
    return index;
  }

  int8_t waitResponse(uint32_t timeout_ms, GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    String data;
    return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
  }

  int8_t waitResponse(GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

 public:
  Stream& stream;

 protected:
  GsmClientLenaR8* sockets[TINY_GSM_MUX_COUNT];
  const char*      gsmNL = GSM_NL;
  bool             has2GFallback;
  bool             supportsAsyncSockets;
};

#endif  // SRC_TINYGSMCLIENTLENAR8_H_
