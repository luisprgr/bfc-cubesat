/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2020 Luis Alberto Paredes Garcia                          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.  *
 *                                                                          *
 ***************************************************************************/

// Código para el receptor del CubeSat

#include <RFM69.h>
#include <SPI.h>

#define NETWORKID 111   
#define MYNODEID 2
#define CLAVE "BFC*$8LAPG<>0020" 
#define FREQUENCY RF69_433MHZ


RFM69 radio(10,2,true);

void setup() {
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.encrypt(CLAVE);
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
}

char *buff;
byte ack=0;

void loop() {
  if (Serial.available() > 0)
  {
    String entrada = Serial.readString();
    buff = malloc(50);
    int tamanioEnvio = 0;
    entrada.toCharArray(buff,50);
    Serial.print(buff);
    tamanioEnvio = strlen(buff);
    Serial.println(tamanioEnvio);
    radio.sendWithRetry(1, buff, tamanioEnvio);
    delay(3);
    free(buff);
  }
  
  if (radio.receiveDone())
  {
    char buffrec[radio.DATALEN];
    strcpy(buffrec,(char *)radio.DATA);
    Serial.println(buffrec);
    Serial.println("Fuerza de señal: ");
    Serial.println (radio.readRSSI()); 
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }
  }
}
