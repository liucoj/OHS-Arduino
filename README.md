# OHS Arduino
 OpenHayStack Arduino sketch

This sketch is based upon OpenHayStack main.c Firmware code for ESP32.
https://github.com/seemoo-lab/openhaystack


# Setup
- Open .ino in Arduino IDE
- EDIT actual adv key on line 27 with your key. Follow padding as shown on example (0x05)

Is it possibile to get a key creating a new accessory in OHS app e copy its advertising key.
To generate a .key file with correct padding it'd useful what's on flash_esp.sh script on OHS main project.

    echo "$PUBKEY" | python3 -m base64 -d - > "$KEYFILE" where PUBKEY is your adv_key and KEYFILE is your tmp.key blank file.

# What is working
- Blinking RED LED (1sec on, 10 sec off)
- LCD initialization with some text printed.

# What's next
- Battery icon
- M5 button for lcd on/off
- Right button for some infos
- BT Paring 

![M5StickC](https://user-images.githubusercontent.com/14237462/122478870-76e51480-cfca-11eb-9ce0-593bfe8623aa.gif)


