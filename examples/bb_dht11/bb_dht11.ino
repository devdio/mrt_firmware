#include <DHT11.h>
// DHT11 by Dhruba Saha
// is installed

DHT11 dht11(10);

void setup() {
    Serial.begin(115200);
}

void loop() {
    int temperature = 0;
    int humidity = 0;

    dht11.setDelay(500);
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C\tHumidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    } else {
           Serial.println(DHT11::getErrorString(result));
    }
}