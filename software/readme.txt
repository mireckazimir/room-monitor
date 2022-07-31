Multisenzorový modul pro Raspberry Pi Zero
Bc. Miroslav Kažimír (xkazim00)

Adresár "room_monitor" obsahuje obslužný softvér pre monitor miestnosti.

Adresár obsahuje:
- knižnicu DFRobot_VEML7700
- knižnicu MCP342x,
- hlavný obslužný program main.cpp
- Makefile na preklad zdrojových kódov
- preložený zdrojový kód v binárnej podobe pre Raspberry Pi Zero

Softvér vyžaduje lokálnu inštaláciu knižnice pigpio, ktorú je možné
inštalovať prostredníctvom "sudo apt install pigpio" a knižnicu mosquitto,
ktorú je taktiež možné inštalovať príkazom "sudo apt install mosquitto".

Podadresár "mic" obsahuje potrebné súbory a pokyny k testu mikrofónov.