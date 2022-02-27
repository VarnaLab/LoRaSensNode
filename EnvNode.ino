#include "boards.h"
#include "loramac.h"
#include "SDS011.h"
#include "bmp280.h"

int counter = 0;

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("LoRa TTN");
    //setupLMIC();

    setupBMP();

    setupSDS();
}

void loop()
{
    Serial.print("LOOP: ");
    Serial.println(counter);

    //loopLMIC();

    loopBMP();

    loopSDS();

#ifdef HAS_DISPLAY
    if (u8g2) {
        char buf[256];
        u8g2->clearBuffer();
        u8g2->drawStr(0, 12, "Transmitting: OK!");
        snprintf(buf, sizeof(buf), "Sending: %d", counter);
        u8g2->drawStr(0, 30, buf);
        u8g2->sendBuffer();
    }
#endif
    counter++;
    delay(5000);
}