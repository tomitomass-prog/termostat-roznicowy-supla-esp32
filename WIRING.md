# Polaczenia

| Funkcja | ESP32 |
|---|---|
| DS18B20 DQ (oba czujniki) | GPIO4 |
| Pull-up 1-Wire | 4.3-4.7 kOhm z GPIO4 do 3.3 V |
| OLED SDA | GPIO21 |
| OLED SCL | GPIO22 |
| Wyjscie przekaznika | GPIO26 |
| Przycisk ekran/tryb | GPIO27 do GND |
| Konfiguracja SUPLA | GPIO0 / BOOT |

Oba DS18B20 pracuja na jednej magistrali. VCC czujnikow podlacz do 3.3 V, GND do GND.

GPIO26 steruje wejsciem modulu przekaznikowego. Nie podlaczaj cewki przekaznika bezposrednio do ESP32.
