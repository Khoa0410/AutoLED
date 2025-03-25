#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

//==== WiFi settings ====
const char* wifi_ssids[] = { "Khoa Khoa", "Galaxy A224103" };
const char* wifi_passwords[] = { "12345678", "khoakhoa" };
const int wifi_count = sizeof(wifi_ssids) / sizeof(wifi_ssids[0]);

//==== MQTT settings ====
const char* mqtt_server = "49b6dcd6236247be8bcfe1416017e3b6.s1.eu.hivemq.cloud";
const char* mqtt_username = "group15_iot";
const char* mqtt_password = "Group15@iot";
const int mqtt_port = 8883;
const char* id = "67e31110a0e08c0712e1db36";

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
const char* topic = "LED";
const char* re_topic = "LED/receive";
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
#define PIR_PIN 13  // Chân kết nối với HC-SR501
#define LED_PIN 12  // Chân kết nối với LED

bool is_control = false; // LED có đang được điều khiển?
bool motion = false; // trạng thái cảm biến
bool prev_motion = false; // trạng thái cũ của cảm biến

//==== Task Handles ====
TaskHandle_t SensorTaskHandle;
TaskHandle_t NetworkTaskHandle;


//==== Điều khiển LED và cảm biến ====
void SensorTask(void* parameter) {
  while (1) {
    if (is_control) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    } else {
      motion = digitalRead(PIR_PIN);  // Đọc trạng thái cảm biến
      if(motion != prev_motion) xTaskNotifyGive(NetworkTaskHandle); // Gửi tín hiệu đến NetworkTask khi cảm biến thay đổi
      prev_motion = motion;

      if (motion == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Phát hiện chuyển động! LED sáng.");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("Không có chuyển động! LED tắt.");
      }

      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

//==== Duy trì kết nối WiFi và MQTT ====
void NetworkTask(void* parameter) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
      connectWiFi();
    }

    if (!client.connected()) {
      Serial.println("MQTT disconnected! Reconnecting...");
      reconnect();
    }
    client.loop();

    // Nếu nhận tín hiệu từ SensorTask, gửi MQTT
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
      publishMessage(topic, true);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//==== Kết nối WiFi ====
void connectWiFi() {
  WiFi.mode(WIFI_STA);

  for (int i = 0; i < wifi_count; i++) {
    Serial.print("Connecting to SSID: ");
    Serial.println(wifi_ssids[i]);

    WiFi.begin(wifi_ssids[i], wifi_passwords[i]);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to " + String(wifi_ssids[i]));
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return;
    } else {
      Serial.println("\nFailed to connect to " + String(wifi_ssids[i]));
    }
  }
}

//==== Kết nối MQTT ====
void reconnect() {
  int attempts = 0;
  while (!client.connected() && attempts < 10) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      client.subscribe(re_topic);
      return;
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      vTaskDelay(pdMS_TO_TICKS(1000));
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
  while (attempts < 10) {
    if (client.publish(topic, jsonBuffer, retained)) {
      Serial.println("Message published [" + String(topic) + "]: " + String(jsonBuffer));
      return;
    } else {
      attempts++;
      vTaskDelay(pdMS_TO_TICKS(1000));
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
  pinMode(PIR_PIN, INPUT);  // Chân cảm biến là INPUT
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.println("Serial connected");

  connectWiFi();
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  if (!client.connected()) reconnect();
  client.loop();

  // Khởi tạo Task cảm biến trên Core 0
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, NULL, 1, &SensorTaskHandle, 0);

  // Khởi tạo Task mạng trên Core 1
  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", 4096, NULL, 1, &NetworkTaskHandle, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // Loop không làm gì vì tất cả chạy trong Task
}
