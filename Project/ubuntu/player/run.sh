ifconfig wlan0 down
iw dev wlan0 set monitor otherbss fcsfail
ifconfig wlan0 up
iwconfig wlan0 channel 9
./player