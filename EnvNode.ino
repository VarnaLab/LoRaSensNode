#include "LMIC-node.h"
#include "SDS011.h"
#include "bmp280.h"

void setup()
{
    setupLMIC();

    setupBMP();

    setupSDS();
 }

void loop()
{
    loopBMP();

    loopSDS();

    loopLMIC();
}