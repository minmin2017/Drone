#pragma once
#include <Arduino.h>
#include <WiFiClient.h>

class MonitorRegistry {
public:
  // Getter คืนค่าเป็น String เพื่อให้ print ได้ง่าย
  using GetterFn = String (*)();

  struct Item {
    const char* name;
    GetterFn getter;
    bool enabled;
  };

  MonitorRegistry() : _count(0), _periodMs(200), _lastTick(0) {}

  // ====== 1) ลงทะเบียนตัวแปรตอนเริ่มต้น ======
  // คืนค่า id (เริ่มจาก 0), ถ้าเต็มคืน -1
  int add(const char* name, GetterFn getter, bool enabledByDefault = false) {
    if (_count >= MAX_ITEMS) return -1;
    _items[_count] = {name, getter, enabledByDefault};
    return _count++;
  }

  // ตั้งค่า period การพ่นข้อมูล (กัน spam)
  void setPeriodMs(uint32_t ms) { _periodMs = ms; }

  // ====== 2) เปิด/ปิดตามเลข id ======
  bool enableById(int id, bool on) {
    if (!validId(id)) return false;
    _items[id].enabled = on;
    return true;
  }

  bool toggleById(int id) {
    if (!validId(id)) return false;
    _items[id].enabled = !_items[id].enabled;
    return true;
  }

  void disableAll() {
    for (int i = 0; i < _count; i++) _items[i].enabled = false;
  }

  void enableAll() {
    for (int i = 0; i < _count; i++) _items[i].enabled = true;
  }

  // ====== 3) ค้นหา id ด้วยชื่อ (เผื่ออยากสั่งด้วยชื่อ) ======
  int findIdByName(const String& name) const {
    for (int i = 0; i < _count; i++) {
      if (name.equalsIgnoreCase(_items[i].name)) return i;
    }
    return -1;
  }

  // ====== 4) พิมพ์รายการทั้งหมด (catalog) เพื่อรู้เลข id ======
  void printCatalog(WiFiClient& client) const {
    client.println("MON CATALOG:");
    client.println("id | en | name");
    for (int i = 0; i < _count; i++) {
      client.printf("%2d |  %d | %s\n", i, _items[i].enabled ? 1 : 0, _items[i].name);
    }
    client.printf("Total: %d\n", _count);
  }

  // ====== 5) พิมพ์เฉพาะตัวที่เปิดอยู่ (enabled) ======
  void printEnabledOnce(WiFiClient& client) const {
    client.println("MON DATA:");
    for (int i = 0; i < _count; i++) {
      if (_items[i].enabled && _items[i].getter) {
        // รูปแบบ: id:name=value
        client.printf("%d:%s=%s\n", i, _items[i].name, _items[i].getter().c_str());
      }
    }
  }

  // ====== 6) พิมพ์ตามเวลา (เรียกใน loop) ======
  void tick(WiFiClient& client) {
    if (_periodMs == 0) return;
    uint32_t now = millis();
    if (now - _lastTick < _periodMs) return;
    _lastTick = now;
    printEnabledOnce(client);
  }

  // ====== 7) คำสั่งควบคุม monitor แบบข้อความ ======
  // ตัวอย่างคำสั่ง:
  // "MON LIST"
  // "MON ON 3"
  // "MON OFF 3"
  // "MON TOGGLE 3"
  // "MON ONALL"
  // "MON OFFALL"
  // "MON PERIOD 200"
  // "MON SHOW"
  void handleCommand(WiFiClient& client, const String& line) {
    String s = line;
    s.trim();
    if (!s.startsWith("MON")) return;

    // แยก token ง่ายๆ
    // MON <cmd> <arg>
    String cmd = token(s, 1);
    cmd.toUpperCase();

    if (cmd == "LIST" || cmd == "CAT" || cmd == "CATALOG") {
      printCatalog(client);
      return;
    }

    if (cmd == "SHOW") {
      printEnabledOnce(client);
      return;
    }

    if (cmd == "ONALL") {
      enableAll();
      client.println("OK MON ONALL");
      return;
    }

    if (cmd == "OFFALL") {
      disableAll();
      client.println("OK MON OFFALL");
      return;
    }

    if (cmd == "PERIOD") {
      long ms = token(s, 2).toInt();
      if (ms < 0) ms = 0;
      setPeriodMs((uint32_t)ms);
      client.printf("OK MON PERIOD %lu\n", (unsigned long)_periodMs);
      return;
    }

    if (cmd == "ON" || cmd == "OFF" || cmd == "TOGGLE") {
      String arg = token(s, 2);
      int id = arg.toInt();
      bool ok = false;

      if (cmd == "ON") ok = enableById(id, true);
      else if (cmd == "OFF") ok = enableById(id, false);
      else ok = toggleById(id);

      if (ok) client.printf("OK MON %s %d\n", cmd.c_str(), id);
      else client.printf("ERR MON bad id: %s\n", arg.c_str());
      return;
    }

    client.println("ERR MON unknown cmd");
  }

private:
  static constexpr int MAX_ITEMS = 40;
  Item _items[MAX_ITEMS];
  int _count;

  uint32_t _periodMs;
  uint32_t _lastTick;

  bool validId(int id) const { return id >= 0 && id < _count; }

  // token(s, index): index=0 คือ "MON", index=1 คือคำสั่ง, index=2 คือ arg
  static String token(const String& s, int index) {
    int start = 0;
    int cur = 0;
    for (int i = 0; i <= s.length(); i++) {
      if (i == s.length() || s[i] == ' ') {
        if (i > start) {
          if (cur == index) return s.substring(start, i);
          cur++;
        }
        start = i + 1;
      }
    }
    return "";
  }
};
