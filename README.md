# ESP32 Radio Controller for my Cardboard Robot
An RC transmitter and receiver using two ESP32, based on ESPNOW

### Overview
The Transmitter encodes the position of the joystick and sends it via ESPNOW protocol to the Receiver.
The receiver: 
* decodes the position of the joystick and activate the two motors properly: Forward Backward Left Right
* generates different light patterns on the 12 LEDs ring (rainbow and similars) and on the two LED eyes.

### Schematics

(ciao )[docs/ESP32-cardboard-robot-rx.png]





