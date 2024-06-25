#include "ADS124S06.h"

ADS124S06 ads;

// Initialize register values according to datasheet recommendations
void setupRegisters()
{
    // Initialize the ADS124S06 with the desired settings
    ads.restoreRegisterDefaults();

    // Set the internal reference voltage
    ads.writeSingleRegister(REG_ADDR_REF, ADS_REFSEL_INT | ADS_REFINT_ON_ALWAYS);

    // Set the IDAC current to 10µA on the AIN0 pin
    ads.writeSingleRegister(REG_ADDR_IDACMAG, ADS_IDACMAG_10);
    ads.writeSingleRegister(REG_ADDR_IDACMUX, ADS_IDAC1_A0 | ADS_IDAC2_OFF);

    // Set the input multiplexer to measure AIN1 referenced to AINCOM
    ads.writeSingleRegister(REG_ADDR_INPMUX, ADS_P_AIN1 | ADS_N_AINCOM);

    // Other necessary configurations can be added here
}

void setup()
{
    // Start serial communication for debugging
    Serial.begin(9600);

    // Initialize the ADS124S06
    ads.begin();
    setupRegisters();

    // Start conversions
    ads.startConversions();
}

void loop()
{
    uint8_t status[1];
    int32_t result;

    // Wait for data ready (DRDY) indication
    // Assuming a function or a way to wait until data is ready
    // Placeholder delay - replace with actual DRDY check if available
    delay(10);

    // Read the conversion result
    result = ads.readConvertedData(status, DIRECT);

    // Print the result
    Serial.print("ADC Result: ");
    Serial.println(result);

    // Wait before reading again
    delay(1000);
}
