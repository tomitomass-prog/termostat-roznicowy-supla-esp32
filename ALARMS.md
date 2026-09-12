# Maska alarmow

Kanal alarmowy w SUPLA publikuje sume bitow:

- 1 - bledna / niepelna konfiguracja adresow DS18B20
- 2 - blad czujnika T1
- 4 - blad czujnika T2
- 8 - aktywny tryb FAILSAFE
- 16 - T1 ponizej Tmin
- 32 - T2 osiagnelo Tmax

Przy bledzie czujnika i domyslnym `FAILSAFE=ON` typowa wartosc bedzie np. 10 (T1 blad + FAILSAFE) albo 14 (oba czujniki blad + FAILSAFE).
