/*
MIT License

Copyright (c) 2018 esp-rfid Community
Copyright (c) 2017 Ömer Şiar Baysal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#define VERSION "2.0.0-esp32c3"

#include "Arduino.h"
#include <WiFi.h>
#include <SPI.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TimeLib.h>
#include <time.h>
#include <MQTTClient.h>
#include <Bounce2.h>
#include "src/magicnumbers.h"
#include "src/config.h"

Config config;

#include <MFRC522.h>
#include "src/PN532.h"
#include <Wiegand.h>
#include "src/rfid125kHz.h"
#include <HardwareSerial.h>

MFRC522 mfrc522 = MFRC522();
PN532 pn532;
WIEGAND wg;
RFID_Reader RFIDr;
HardwareSerial *rdm6300HwSerial = NULL;

// relay specific variables
bool activateRelay[MAX_NUM_RELAYS] = {false, false, false, false};
bool deactivateRelay[MAX_NUM_RELAYS] = {false, false, false, false};

// these are from vendors
#include "src/webh/glyphicons-halflings-regular.woff.gz.h"
#include "src/webh/required.css.gz.h"
#include "src/webh/required.js.gz.h"

// these are from us which can be updated and changed
#include "src/webh/esprfid.js.gz.h"
#include "src/webh/esprfid.htm.gz.h"
#include "src/webh/index.html.gz.h"

WiFiClient mqttNet;
MQTTClient mqttClient(2048);

// millis-based timers replacing Ticker
unsigned long mqttReconnectTimerStart = 0;
bool mqttReconnectTimerActive = false;
unsigned long wifiReconnectTimerStart = 0;
bool wifiReconnectTimerActive = false;

// Ticker-like helper for deferred websocket send (replaces wsMessageTicker)
typedef std::function<void(void)> WsTickerCallback;
WsTickerCallback wsTickerCallback = nullptr;
unsigned long wsTickerStart = 0;
bool wsTickerActive = false;

void wsTickerOnce(unsigned long ms, WsTickerCallback cb) {
	wsTickerCallback = cb;
	wsTickerStart = millis();
	wsTickerActive = true;
}

void processWsTicker() {
	if (wsTickerActive && millis() - wsTickerStart >= 50) {
		wsTickerActive = false;
		if (wsTickerCallback) {
			wsTickerCallback();
			wsTickerCallback = nullptr;
		}
	}
}

Bounce openLockButton;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define LEDoff HIGH
#define LEDon LOW

#define BEEPERoff HIGH
#define BEEPERon LOW

// Variables for whole scope
unsigned long cooldown = 0;
unsigned long currentMillis = 0;
unsigned long deltaTime = 0;
bool doEnableWifi = false;
bool formatreq = false;
const char *httpUsername = "admin";
unsigned long keyTimer = 0;
uint8_t lastDoorbellState = 0;
uint8_t lastDoorState = 0;
uint8_t lastTamperState = 0;
unsigned long nextbeat = 0;
time_t epoch;
time_t lastNTPepoch;
unsigned long lastNTPSync = 0;
unsigned long openDoorMillis = 0;
unsigned long previousLoopMillis = 0;
unsigned long previousMillis = 0;
bool shouldReboot = false;
tm timeinfo;
unsigned long uptimeSeconds = 0;
unsigned long wifiPinBlink = millis();
unsigned long wiFiUptimeMillis = 0;

#include "src/led.esp"
#include "src/beeper.esp"
#include "src/log.esp"
#include "src/mqtt.esp"
#include "src/helpers.esp"
#include "src/wsResponses.esp"
#include "src/rfid.esp"
#include "src/wifi.esp"
#include "src/config.esp"
#include "src/websocket.esp"
#include "src/webserver.esp"
#include "src/door.esp"
#include "src/doorbell.esp"

void setup()
{
	// Keep startup diagnostics enabled even when DEBUG is not defined.
	// Required for ESP32-C3 native USB CDC serial troubleshooting.
	Serial.begin(115200);
	unsigned long serialWaitStart = millis();
	while (!Serial && (millis() - serialWaitStart < 1500)) {
		delay(10);
	}
	Serial.println();
	Serial.println(F("[BOOT] ESP-RFID setup entered"));

#ifdef DEBUG
	Serial.print(F("[ INFO ] ESP RFID v"));
	Serial.println(VERSION);

	Serial.printf("Flash chip size: %u bytes\n", ESP.getFlashChipSize());
	Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
	Serial.printf("CPU freq: %u MHz\n", ESP.getCpuFreqMHz());
	Serial.printf("SDK version: %s\n", ESP.getSdkVersion());
#endif

	Serial.println(F("[BOOT] SPIFFS init"));
	if (!SPIFFS.begin(true))
	{
		if (SPIFFS.format())
		{
			writeEvent("WARN", "sys", "Filesystem formatted", "");
		}
		else
		{
			Serial.println(F("[WARN] Could not format filesystem!"));
		}
	}
	Serial.println(F("[BOOT] SPIFFS ready"));

	Serial.println(F("[BOOT] before config load"));
	yield();
	bool configured = false;
	configured = loadConfiguration(config);
	Serial.printf("[BOOT] config loaded: %s\n", configured ? "yes" : "no");
	// Initialize WiFi before networking clients (ESP32/LWIP requires this order).
	setupWifi(configured);
	Serial.println(F("[BOOT] WiFi setup done"));
	// Start NTP only after WiFi/LWIP is initialized.
	configTime(0, 0, config.ntpServer);
	setenv("TZ", config.tzInfo ? config.tzInfo : "UTC", 1);
	tzset();
	Serial.println(F("[BOOT] Time setup done"));
	setupMqtt();
	Serial.println(F("[BOOT] MQTT setup done"));
	setupWebServer();
	Serial.println(F("[BOOT] Webserver setup done"));
	writeEvent("INFO", "sys", "System setup completed, running", "");
	Serial.println(F("[BOOT] setup complete"));
}

void loop()
{
	currentMillis = millis();
	deltaTime = currentMillis - previousLoopMillis;
	uptimeSeconds = currentMillis / 1000;
	previousLoopMillis = currentMillis;
	
	trySyncNTPtime(10);

	openLockButton.update();
	if (config.openlockpin != 255 && openLockButton.fell())
	{
		writeLatest(" ", "Button", 1);
		mqttPublishAccess(epoch, "true", "Always", "Button", " ", " ");
		activateRelay[0] = true;
		beeperValidAccess();
		// TODO: handle other relays
	}

	ledWifiStatus();
	ledAccessDeniedOff();
	beeperBeep();
	doorStatus();
	doorbellStatus();

	// Process millis-based timers (replaces Ticker)
	if (mqttReconnectTimerActive && currentMillis - mqttReconnectTimerStart >= 60000) {
		mqttReconnectTimerActive = false;
		connectToMqtt();
	}
	if (wifiReconnectTimerActive && currentMillis - wifiReconnectTimerStart >= 300000) {
		wifiReconnectTimerActive = false;
		setEnableWifi();
	}
	processWsTicker();

	if ((long)(currentMillis - cooldown) >= 0)
	{
		rfidLoop();
	}

	for (int currentRelay = 0; currentRelay < config.numRelays; currentRelay++)
	{
		if (config.lockType[currentRelay] == LOCKTYPE_CONTINUOUS) // Continuous relay mode
		{
			if (activateRelay[currentRelay])
			{
				if (digitalRead(config.relayPin[currentRelay]) == !config.relayType[currentRelay]) // currently OFF, need to switch ON
				{
					mqttPublishIo("lock" + String(currentRelay), "UNLOCKED");
#ifdef DEBUG
					Serial.print("mili : ");
					Serial.println(millis());
					Serial.printf("activating relay %d now\n", currentRelay);
#endif
					digitalWrite(config.relayPin[currentRelay], config.relayType[currentRelay]);
				}
				else // currently ON, need to switch OFF
				{
					mqttPublishIo("lock" + String(currentRelay), "LOCKED");
#ifdef DEBUG
					Serial.print("mili : ");
					Serial.println(millis());
					Serial.printf("deactivating relay %d now\n", currentRelay);
#endif
					digitalWrite(config.relayPin[currentRelay], !config.relayType[currentRelay]);
				}
				activateRelay[currentRelay] = false;
			}
		}
		else if (config.lockType[currentRelay] == LOCKTYPE_MOMENTARY) // Momentary relay mode
		{
			if (activateRelay[currentRelay])
			{
				mqttPublishIo("lock" + String(currentRelay), "UNLOCKED");
#ifdef DEBUG
				Serial.print("mili : ");
				Serial.println(millis());
				Serial.printf("activating relay %d now\n", currentRelay);
#endif
				digitalWrite(config.relayPin[currentRelay], config.relayType[currentRelay]);
				previousMillis = millis();
				activateRelay[currentRelay] = false;
				deactivateRelay[currentRelay] = true;
			}
			else if ((currentMillis - previousMillis >= config.activateTime[currentRelay]) && (deactivateRelay[currentRelay]))
			{
				mqttPublishIo("lock" + String(currentRelay), "LOCKED");
#ifdef DEBUG
				Serial.println(currentMillis);
				Serial.println(previousMillis);
				Serial.println(config.activateTime[currentRelay]);
				Serial.println(activateRelay[currentRelay]);
				Serial.println("deactivate relay after this");
				Serial.print("mili : ");
				Serial.println(millis());
#endif
				digitalWrite(config.relayPin[currentRelay], !config.relayType[currentRelay]);
				deactivateRelay[currentRelay] = false;
			}
		}
	}
	if (formatreq)
	{
#ifdef DEBUG
		Serial.println(F("[ WARN ] Factory reset initiated..."));
#endif
		SPIFFS.end();
		ws.enable(false);
		SPIFFS.format();
		ESP.restart();
	}

	if (config.autoRestartIntervalSeconds > 0 && uptimeSeconds > config.autoRestartIntervalSeconds)
	{
		writeEvent("WARN", "sys", "Auto restarting...", "");
		shouldReboot = true;
	}

	if (shouldReboot)
	{
		writeEvent("INFO", "sys", "System is going to reboot", "");
		SPIFFS.end();
		ESP.restart();
	}

	if (WiFi.isConnected())
	{
		wiFiUptimeMillis += deltaTime;
	}

	if (config.wifiTimeout > 0 && wiFiUptimeMillis > (config.wifiTimeout * 1000) && WiFi.isConnected())
	{
		writeEvent("INFO", "wifi", "WiFi is going to be disabled", "");
		disableWifi();
	}

	// don't try connecting to WiFi when waiting for pincode
	if (doEnableWifi == true && keyTimer == 0 && activateRelay[0] == true)
	{
		if (!WiFi.isConnected())
		{
			enableWifi();
			writeEvent("INFO", "wifi", "Enabling WiFi", "");
			doEnableWifi = false;
		}
	}

	if (config.mqttEnabled)
	{
		mqttClient.loop();
		if (mqttClient.connected())
		{
			if ((unsigned)epoch > nextbeat)
			{
				mqttPublishHeartbeat(epoch, uptimeSeconds);
				nextbeat = (unsigned)epoch + config.mqttInterval;
#ifdef DEBUG
				Serial.print("[ INFO ] Nextbeat=");
				Serial.println(nextbeat);
#endif
			}
			processMqttQueue();
		}
	}

	processWsQueue();

	// clean unused websockets
	ws.cleanupClients();
}
