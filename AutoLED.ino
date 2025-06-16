#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ESP32Ping.h>

//==== MQTT settings ====
const char* mqtt_server = "49b6dcd6236247be8bcfe1416017e3b6.s1.eu.hivemq.cloud";
const char* mqtt_username = "IoTPlatform";
const char* mqtt_password = "IoT@Platform_215599";
const int mqtt_port = 8883;
char id[40] = "";

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char topic[40] = "";
char re_topic[40] = "";
static const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// GPIO setting
#define PIR_PIN 13     // Chân kết nối với HC-SR501
#define LED_PIN 12     // Chân kết nối với LED
#define BUTTON_PIN 15  // Chân kết nối với BUTTON

bool is_control = false;   // LED có đang được điều khiển?
bool motion = false;       // trạng thái cảm biến
bool prev_motion = false;  // trạng thái cũ của cảm biến
bool is_enable_ap = false;

//==== Task Handles ====
TaskHandle_t SensorTaskHandle;
TaskHandle_t NetworkTaskHandle;

Preferences preferences;
void saveConfigToEEPROM(const char* id, const char* topic, const char* re_topic) {
  preferences.begin("config", false);
  preferences.putString("device_id", id);
  preferences.putString("mqtt_topic", topic);
  preferences.putString("mqtt_retopic", re_topic);
  preferences.end();

  // Debug
  Serial.println("[EEPROM] Saved config:");
  Serial.print("  ID: ");
  Serial.println(id);
  Serial.print("  Topic: ");
  Serial.println(topic);
  Serial.print("  Re-topic: ");
  Serial.println(re_topic);
}

void loadConfigFromEEPROM() {
  preferences.begin("config", true);
  strcpy(id, preferences.getString("device_id", "").c_str());  // Nếu chưa có giá trị, dùng mặc định
  strcpy(topic, preferences.getString("mqtt_topic", "").c_str());
  strcpy(re_topic, preferences.getString("mqtt_retopic", "").c_str());
  preferences.end();

  // Debug
  Serial.println("[EEPROM] Loaded config:");
  Serial.print("  ID: ");
  Serial.println(id);
  Serial.print("  Topic: ");
  Serial.println(topic);
  Serial.print("  Re-topic: ");
  Serial.println(re_topic);
}

//==== Điều khiển LED và cảm biến ====
void SensorTask(void* parameter) {
  while (1) {
    // 🔹 Bấm nút để vào AP Mode
    if (digitalRead(BUTTON_PIN) == LOW) {
      is_enable_ap = true;
      xTaskNotifyGive(NetworkTaskHandle);  // Gửi tín hiệu đến NetworkTask để vào AP Mode
    } else is_enable_ap = false;

    if (is_control) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    } else {
      motion = digitalRead(PIR_PIN);  // Đọc trạng thái cảm biến
      if (motion != prev_motion) {
        xTaskNotifyGive(NetworkTaskHandle);  // Gửi tín hiệu đến NetworkTask khi cảm biến thay đổi
        if (motion == HIGH) Serial.println("Phát hiện chuyển động! Bật RELAY.");
        else Serial.println("Không có chuyển động! Tắt RELAY.");
      }
      prev_motion = motion;

      if (motion == HIGH) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

//==== Duy trì kết nối WiFi và MQTT ====
void NetworkTask(void* parameter) {
  while (1) {
    // Tự động kết nối WiFi và MQTT
    if (WiFi.status() != WL_CONNECTED) {
      is_control = false;  // Tắt chế độ điều khiển qua mạng
      Serial.println("WiFi disconnected! Reconnecting...");
      connectWiFi();
    }

    if (!client.connected()) {
      is_control = false;  // Tắt chế độ điều khiển qua mạng
      Serial.println("MQTT disconnected! Reconnecting...");
      reconnect();
    }
    client.loop();

    // Xử lý nhận tín hiệu từ SensorTask
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
      if (is_enable_ap)
        enableAPMode();
      else
        publishMessage(topic, true);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//==== Kết nối WiFi ====
void connectWiFi() {
  Serial.println("Đang kết nối WiFi...");

  // Nếu đã có WiFi lưu sẵn, thử kết nối lại
  WiFi.begin();
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 5) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi đã kết nối lại!");
    Serial.print("Địa chỉ IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nKhông kết nối lại được với WiFi, thiết bị hoạt động không có mạng.");
  }
}

// Check Internet access for WiFi
bool checkInternet() {
  Serial.println("Checking Internet access by pinging 8.8.8.8...");
  bool success = Ping.ping(IPAddress(8, 8, 8, 8), 3);  // Ping 3 lần đến Google DNS

  if (success) {
    Serial.println("Internet access confirmed");
    return true;
  } else {
    Serial.println("No Internet access");
    return false;
  }
}

//==== Vào AP Mode ====
void enableAPMode() {
  WiFi.disconnect(true);  // Ngắt kết nối WiFi để vào AP Mode
  vTaskDelay(pdMS_TO_TICKS(100));

  WiFi.mode(WIFI_AP);  // Vào AP Mode
  WiFiManager wm;
  wm.setConfigPortalTimeout(120);  // Giới hạn AP trong 120 giây

  loadConfigFromEEPROM();  // Load config hiện tại

  // input field cho id, topic trong WiFi Manager
  WiFiManagerParameter custom_id("device_id", "Device ID", id, 40);
  WiFiManagerParameter custom_topic("mqtt_topic", "MQTT Topic", topic, 40);
  WiFiManagerParameter custom_retopic("mqtt_retopic", "MQTT Receive Topic", re_topic, 40);

  // Thêm các tham số vào WiFiManager
  wm.addParameter(&custom_id);
  wm.addParameter(&custom_topic);
  wm.addParameter(&custom_retopic);

  if (wm.startConfigPortal("AutoLED_Config")) {
    Serial.println("Cấu hình thành công!");
    Serial.print("Địa chỉ IP: ");
    Serial.println(WiFi.localIP());

    // Lưu cấu hình sau khi người dùng nhập dữ liệu mới
    saveConfigToEEPROM(custom_id.getValue(), custom_topic.getValue(), custom_retopic.getValue());
    vTaskDelay(pdMS_TO_TICKS(1000));
    loadConfigFromEEPROM();

    // Tắt AP sau cấu hình
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.softAPdisconnect(true);
      Serial.println("Tắt AP sau khi cấu hình xong.");
    }
  } else {
    Serial.println("Hết thời gian cấu hình hoặc không kết nối được!");
  }
}

//==== Kết nối MQTT ====
void reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Không có WiFi, không thể kết nối MQTT.");
    return;
  }

  if (!checkInternet()) {
    Serial.println("WiFi không có kết nối mạng. Không thể kết nối MQTT");
    return;
  }

  int attempts = 0;
  while (!client.connected() && attempts < 5) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      client.subscribe(re_topic);
      return;
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    attempts++;
  }
}

//==== Gửi dữ liệu MQTT ====
void publishMessage(const char* topic, boolean retained) {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["_id"] = id;
  JsonObject telemetry = jsonDoc.createNestedObject("telemetry");
  telemetry["state"] = motion;

  char jsonBuffer[200];
  serializeJson(jsonDoc, jsonBuffer);

  int attempts = 0;
  while (attempts < 5) {
    if (client.publish(topic, jsonBuffer, retained)) {
      Serial.println("Message published [" + String(topic) + "]: " + String(jsonBuffer));
      return;
    } else {
      attempts++;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
  Serial.println("Failed to publish message");
}

//==== Xử lý nhận dữ liệu từ MQTT ====
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic [");
  Serial.print(topic);
  Serial.print("]: ");

  // Chuyển payload thành chuỗi JSON
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);

  // Parse JSON
  StaticJsonDocument<200> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, message);

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.f_str());
    return;
  }

  // Kiểm tra deviceId có trùng khớp không
  const char* receivedId = jsonDoc["deviceId"];
  if (receivedId && strcmp(receivedId, id) == 0) {
    motion = jsonDoc["state"];
    Serial.print("Updated state: ");
    Serial.println(motion ? "LED ON" : "LED OFF");

    // Điều khiển LED
    if (motion) {
      is_control = true;
      digitalWrite(LED_PIN, HIGH);
      Serial.println("Turn LED ON");
    } else {
      is_control = false;
      prev_motion = false;
      digitalWrite(LED_PIN, LOW);
      Serial.println("Turn LED OFF");
    }
  } else {
    Serial.println("Device ID mismatch, ignoring message.");
  }
}

void setup() {
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.println("Serial connected");

  // Đọc cấu hình đã lưu trong EEPROM
  loadConfigFromEEPROM();

  connectWiFi();
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  if (!client.connected()) reconnect();
  client.loop();

  // Khởi tạo Task cảm biến trên Core 0
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 1, &SensorTaskHandle, 0);

  // Khởi tạo Task mạng trên Core 1
  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", 8192, NULL, 1, &NetworkTaskHandle, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // Loop không làm gì vì tất cả chạy trong Task
}