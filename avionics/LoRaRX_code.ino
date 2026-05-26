#include <SPI.h>
#include <LoRa.h>

// ---------------- LoRa Pins ----------------
#define LORA_SS   23
#define LORA_RST  4
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// Custom SPI pins
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 27

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== LoRa RX Ready ===");

  // Initialize SPI bus with custom pins
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed! Check wiring and power.");
    while (1);
  }

  Serial.println("LoRa init successful. Waiting for packets...");
}

void loop() {
  // Check for incoming packet
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    // Print the full CSV line
    Serial.println(received);
  }
}
