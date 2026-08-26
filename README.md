Hardware yang dibutuhkan
Komponen yang saya pakai cukup standar:
- ESP32C3 Super Mini Dev Module
- Modul GPS NEO-6M (atau yang kompatibel, NMEA 9600 baud)
- Radio HT VHF
- Optocoupler (4N35) untuk trigger PTT
- Kabel audio 3.5mm / Kabel headset HT
- Kabel jumper dan breadboard / PCB
- Power supply 5V

Perakitan Hardware
Perakitan silakan lihat pada gambar. Berikut langkah-langkahnya:
1. Hubungkan GPS
  TX modul GPS → GPIO6
  VCC GPS → 3.3V ESP32C3 Super Mini
  GND GPS → GND ESP32C3 Super Mini
  (RX GPS tidak perlu disambung karena ESP32C3 Super Mini hanya membaca data posisi, tidak mengirim perintah ke GPS).
2. Hubungkan output AFSK
   GPIO10 ESP32C3 Super Mini → satu kabel ke input mic radio HT (lewat jack audio)
   GND ESP32 Super Mini → GND jack audio
3. GND ESP32C3 Super Mini → GND jack audio.
   Hubungkan PTT dengan optocoupler Saya pakai optocoupler supaya lebih aman dan tidak langsung nyambung ground radio ke ESP32.
   Pin GPIO5 ESP32C3 Super Mini → pin 1 (anoda LED) optocoupler
   Pin 2 (katoda LED) optocoupler → GND ESP32C3 Super Mini
   Pin 4 (emitter) optocoupler → GND mic radio
   Pin 5 (collector) optocoupler → pin PTT mic radio (biasanya yang biasa dipakai untuk push-to-talk)
