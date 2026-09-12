# Termostat roznicowy ON/OFF - ESP32 + SUPLA

Sterownik przeznaczony glownie do ochrony bojlera przed wychladzaniem przez chlodniejszy czynnik.

## Logika AUTO

- T1 = temperatura zrodla / czynnika przed bojlerem.
- T2 = temperatura w bojlerze / odbiorniku.
- `T1 >= T2 + DeltaON` -> ON.
- `T1 <= T2 + DeltaOFF` -> OFF.
- `T1 < Tmin` -> OFF.
- `T2 >= Tmax` -> OFF.
- Pomiedzy `DeltaOFF` i `DeltaON` zapamietywany jest poprzedni stan.
- Uszkodzenie jednego lub obu skonfigurowanych DS18B20 -> domyslnie ON (FAILSAFE).
- Brak / bledny adres DS18B20 w konfiguracji -> OFF, aby niezaprogramowane urzadzenie nie zalaczylo wyjscia.

## SUPLA

Kanaly sa tworzone w kolejnosci:

1. Nastawa `Tmin` - termostat, domyslnie 30 C.
2. Nastawa `Tmax` - termostat, domyslnie 70 C.
3. Temperatura T1.
4. Temperatura T2.
5. `AUTO` - ON = automatyka roznicowa, OFF = tryb reczny.
6. `RECZNY` - stan wyjscia w trybie recznym.
7. Roznica temperatur T1-T2.
8. Stan wyjscia 0/1.
9. Maska alarmow.
10. Tryb: 0=AUTO, 1=MAN OFF, 2=MAN ON, 3=FAILSAFE.

Po dodaniu urzadzenia w SUPLA warto nadac kanalom czytelne nazwy.

## Additional settings

W panelu lokalnym ESP32 ustaw:

- `DS18B20 czujnik 1 / zrodlo (16 HEX)`
- `DS18B20 czujnik 2 / bojler (16 HEX)`
- `Roznica zalaczenia DeltaON` (domyslnie 2.0 C)
- `Roznica wylaczenia DeltaOFF` (domyslnie 0.5 C)
- `Histereza limitow Tmin/Tmax` (domyslnie 0.5 C)
- `Blad czujnika: 1=ON 0=OFF` (domyslnie 1)
- `Przekaznik: 1=aktywny HIGH 0=aktywny LOW`

Adres DS18B20 musi miec pelne 16 znakow HEX, np. `28EC82CE10000064`.

## Przycisk GPIO27

- krotkie nacisniecie: zmiana strony OLED,
- dlugie przytrzymanie: cykl `AUTO -> MAN ON -> MAN OFF -> AUTO`.

## Kompilacja

Projekt jest przygotowany dla PlatformIO/pioarduino i SUPLA Device v26.4.
GitHub Actions generuje `termostat-roznicowy-merged.bin` do programowania calego flash od adresu `0x0` oraz `termostat-roznicowy-app.bin`.
