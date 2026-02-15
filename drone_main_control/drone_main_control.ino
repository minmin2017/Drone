#include "Communication_order.h"
// ---------------- WiFi ----------------
const char* ssid     = "MinMin2017";
const char* password = "minmin2017i";
const bool wifi = true;

LanCommand net;
void setup() {
  // put your setup code here, to run once:
  if (wifi) {
    bool ok = net.begin(ssid, password);
    if (!ok) {
      Serial.println("WiFi failed (net.begin timeout)");
      while (1) { delay(1000); }
    }
    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
  }
  net.onA(doA);
  net.onB(doB);
  net.onC(doC);
  net.onReset(doReset);
}

void loop() {
  // put your main code here, to run repeatedly:

}
