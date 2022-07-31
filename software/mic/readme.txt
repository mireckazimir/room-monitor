Multisenzorový modul pro Raspberry Pi Zero
Bc. Miroslav Kažimír (xkazim00)

Adresár "mic" obsahuje potrebné pokyny k testu mikrofónov.

Pre inštaláciu obslužného softvéru mikrofónu je nevyhnutné,
aby mal systém inštalovaný Python verzie 3 a Pip:

sudo apt install python3-pip

Následne je potrebné zadať v termináli tieto príkazy:

cd ~
sudo pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/i2smic.py
sudo python3 i2smic.py

Posledný z týchto príkazov nainštaluje ovládač pre mikrofóny.
V priebehu inštalácie sa užívateľa opýta, či chce, aby bol
ovládač načítaný automaticky pri štarte systému. Po skončení
inštalácie sa tiež opýta, či má systém reštartovať a uplatniť
tak nové nastavenia.

Pred použitím mikrofónu je potrebné zistiť jeho číslo zvukovej karty.
K tomu je možné využiť príkaz:

arecord -l

Samotné nahrávanie je potom možné vykonať pomocou príkazu:

arecord -D plughw:0 -c2 -r 48000 -f S32_LE -t wav -V stereo -v file_stereo.wav

Prehratie nahraného zvuku je možné vykonať príkazom:

aplay file_stereo.wav