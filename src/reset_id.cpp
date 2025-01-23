#include <globals.h>
#include <reset_id.h>
#include <stdint.h>
#include <eeprom_constants.h>

void resetId() {
    // Send the new id packet.
    char new_id_packet = 128;
    client.write(&new_id_packet, 1);

    // Receive the created id packet
    uint8_t id_data[3] = {0};
    while (client.available() < 3);
    client.readBytes(reinterpret_cast<uint8_t*>(&id_data), 3);

    if (id_data[0] != 129) {
        Serial.printf("Invalid packet id for new id: %d", id_data[0]);
        return;
    }
    
    // Bit twiddle the packet into an id.
    uint16_t id = (id_data[1] << 8) | id_data[2];
    Serial.printf("Received new id: %d\n", id);

    // Update the EEPROM with the new ID
    EEPROM.writeUShort(ID_ADDRESS, id);
    EEPROM.commit();
    uint16_t check_id = EEPROM.readUShort(ID_ADDRESS);

    // Double check the value was written correctly.
    if (id != check_id) {
        Serial.printf("Failed to write id: %d, read: %d\n", id, check_id);
    } else {
        Serial.printf("Wrote new id.\n");
    }

    // Wait for the reset button to be deasserted.
    while (digitalRead(RESET_ID));
    ESP.restart();
}