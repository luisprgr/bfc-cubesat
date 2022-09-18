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

// Código para el CubeSat Base

#include <avr/wdt.h>                          //Libreria para configurar el watchdog del microprocesador
#include <RFM69.h>                            //Libreria para el control del modulo RFM69HCW
#include <SPI.h>                              //Libreria para comunicarse con dispositivos SPI
#include <Wire.h>                             //Libreria para comunicarse con dispositivos I2C
#include <SparkFunTMP102.h>                   //Libreria para facilitar el uso de sensores Texas Instruments TMP102

//-------------------------------------------------------------------------------------------------------
//--------------------Configuración del modulo de comunicación-----------------------

#define RED 111                               // ID de red en la que se encontrará
#define ID 1                                  // ID del CubeSat en la red
#define RECEPTOR 2                            // ID del nodo en la red que recibirá la comunicación
#define CLAVE "BFC*$8<>0020"                  // Clave para encriptar los paquetes en la red máximo 16 caracteres

      // Ya que la contraseña se mostrara en la documentación, esta debe ser cambiada

//----------------Fin de Configuración del modulo de comunicación--------------------
//-------------------------------------------------------------------------------------------------------
//--------------Configuración espera antes de realizar alguna tarea------------------

// #define TIEMPOESPERA 10000                    //Según el estándar CubeSat se deben esperar mínimo 45 minutos antes de cualquier transmisión de datos, se han puesto 10 segundos para demostración del prototipo (El valor va en milisegundos)
#define TIEMPOESPERARESISTENCIA 3000          //Tiempo en milisegundos durante el cual se calentara la resistencia que controla el despliegue de las antenas
#define TIEMPOESPERAALERTATEMPERATURA 10000   //Tiempo a esperar para volver a la ejecución normal del código, luego de una alerta de alta temperatura
#define PERIODOMEDICIONTEMPERATURA 1000       //Tiempo en milisegundos antes de tomar otra medida de los sensores de temperatura
#define PERIODOMEDICIONCORRIENTE 1000         //Tiempo en milisegundos antes de tomar otra medida de los sensores de corriente
unsigned long periodoTransmision = 5000;      //Tiempo en milisegundos antes de volver a transmitir señales, se han puesto 10 segundos ya que es el mínimo tiempo de espera para transmisiones periódicas establecido en la Norma Técnica de Espectro de uso Libre y de Espectro para Uso Determinado en Bandas Libres (Norma expedida por la ARCOTEL en el año 2018) 
#define NCOMANDOSNOVALIDOSANTESDEREINICIO 10  //Numero de comandos no validos recibidos antes de reiniciar microcontrolador
                                              
      //El periodo de transmisión puede ser cambiado luego mediante comandos

//--------Fin Configuración tiempo de espera antes de realizar alguna tarea----------
//-------------------------------------------------------------------------------------------------------
//-------------Configuración sensibilidad para los sensores de corriente-------------

#define SENSIBILIDADSENSORCORRIENTE3V 0.185   //Sensibilidad para el sensor en la linea de 3.3v
#define SENSIBILIDADSENSORCORRIENTE5V 0.185   //Sensibilidad para el sensor en las lineas de 5v

//----------Fin Configuracion sensibilidad para los sensores de corriente------------
//-------------------------------------------------------------------------------------------------------

#define PIN_ALERTA_TEMP_CPU 6                 //Pin donde esta el pin de alerta del sensor de temperatura de la pcb del microprocesador
#define PIN_ALERTA_TEMP_BATT1 35              //Pin donde esta el pin de alerta del sensor de temperatura de la batería 1
#define PIN_ALERTA_TEMP_BATT2 36              //Pin donde esta el pin de alerta del sensor de temperatura de la batería 2

TMP102 sensorCPU(0x4B);                       //Dirección I2C del sensor de temperatura que esta en la pcb del microprocesador
TMP102 sensorBATT1(0x4A);                     //Dirección I2C del sensor de temperatura de la batería 1
TMP102 sensorBATT2(0x49);                     //Dirección I2C del sensor de temperatura de la batería 2

RFM69 radio(53,2,true);                       //Inicialización del objeto con el que se manejara el modulo RFM69

void setup() {

  //Configuración del pin desde el cual se maneja el mosfet que controla el paso de corriente a la resistencia que mantiene las antenas recogidas
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
  
  //Se envía un 1 lógico (se da un voltaje igual al de la linea de 5v) a la puerta del mosfet (el mosfet es de canal P, por lo que bajo este voltaje no permite el paso de corriente)

  //Configuración de Watchdog
  wdt_disable(); //Se desactiva el watchdog para evitar reinicios durante su configuración
  wdt_enable(WDTO_8S);  //Se vuelve a activar el circuito watchdog y se lo configura para que reinicie el procesador cada 8 segundos a menos que se reinicie el contador
  
  //Inicialización RFM69
  radio.initialize(RF69_433MHZ, ID, RED);
  radio.encrypt(CLAVE);

  //Configuración de modo de los pines donde están los sensores de temperatura
  pinMode(PIN_ALERTA_TEMP_CPU,INPUT);
  pinMode(PIN_ALERTA_TEMP_BATT1,INPUT);
  pinMode(PIN_ALERTA_TEMP_BATT2,INPUT);

  //Inicio de comunicación I2C
  Wire.begin();  

  //Configuración de los sensores de temperatura
  sensorCPU.begin(); //Inicio del sensor
  sensorCPU.setHighTempC(100.0); //Temperatura a partir de la cual se darán alertas de temperatura (valores mayores o iguales a 100 grados centígrados en este caso)
  sensorCPU.setLowTempC(0.0); //Temperatura a partir de la cual se darán alertas de temperatura (valores menores o iguales a 0 grados centígrados en este caso)
  sensorCPU.setFault(2); //Mínimo de veces que el sensor debe detectar las temperaturas de alerta antes de dar la alerta, valores que acepta la función 0) 1 vez, 1) 2 veces, 2) 4 veces, 3) 6 veces
  sensorCPU.setAlertPolarity(1); //Lógica del pin de interrupción de la alarma, valores que acepta la función 0) cuando la alarma se active el pin dará un 0 lógico, 1) cuando la alarma se active el pin dará un 1 lógico 
  sensorCPU.setAlertMode(0);  //Selecciona el modo en el que debe estar el sensor, valores que acepta la función 0) Modo Comparador, 1) modo de interrupción
  
  //Cuando el sensor detecta que la temperatura excede la temperatura maxima configurada o está por debajo de la temperatura minima configurada, 
  //en modo comparador se activa el pin de alerta y se mantiene activado hasta que el sensor sale de esas temperaturas, 
  //en modo interrupción se activa el pin de alerta y se desactiva una vez que el microcontrolador ha leído la temperatura
  
  sensorCPU.setConversionRate(2); //Indica que tan rápido el sensor lee la temperatura, valores que acepta la función 0) 0.25Hz, 1) 1Hz, 2) 4Hz, 3) 8Hz
  sensorCPU.setExtendedMode(1); //Activar o desactivar modo extendido para la lectura de temperatura, valores que acepta la función 
  //0) desactiva el modo extendido, el formato de datos con el que se lee la temperatura es de 12 bits (el rango de temperaturas va de -55 grados centígrados a 128 grados centígrados)
  //1) activa el modo extendido, el formato de datos con el que se lee la temperatura es de 13 bits (el rango de temperaturas va de -55 grados centígrados a 150 grados centígrados)
  
  sensorBATT1.begin();
  sensorBATT1.setHighTempC(100.0);
  sensorBATT1.setLowTempC(0.0);
  sensorBATT1.setFault(2);
  sensorBATT1.setAlertPolarity(1);
  sensorBATT1.setAlertMode(0);
  sensorBATT1.setConversionRate(2);
  sensorBATT1.setExtendedMode(1);
  
  sensorBATT2.begin();
  sensorBATT2.setHighTempC(100.0);
  sensorBATT2.setLowTempC(0.0);
  sensorBATT2.setFault(2);
  sensorBATT2.setAlertPolarity(1);
  sensorBATT2.setAlertMode(0);
  sensorBATT2.setConversionRate(2);
  sensorBATT2.setExtendedMode(1); 
}

//Variables para guardar valores del consumo de corriente
double i5 = 0;
double i5prom = 0;
double i3 = 0;
double i3prom = 0;

//Variables para guardar valores de consumo de corriente en arreglos de caracteres
char *corriente3v;
char *corriente5v;
char *corriente3vprom;
char *corriente5vprom;

//Variables para guardar temperaturas en arreglos de caracteres
char *temperaturaCPU;
char *temperaturaBATT1;
char *temperaturaBATT2;
char *temperaturaRFTR;

//Punteros para guardar el buffer de datos a enviar 
char *buff;

//Tamaño del buffer de datos transmitido, el tamaño máximo es de 50 bytes
int tamanioBuffer = 0;

//Variable para guardar el tiempo de pausa de transmisión de señales
int pausa = 0;

//Contador de comandos utilizados
int comandos = 0;
int comandosTemperatura = 0;
int comandosConsumo = 0;
int comandosPausa = 0;
int comandosNoValidos = 0;
int comandosNoValidosConsecutivos = 0;

//Variables para controlar diferentes estados en el código
bool comunicacion = true;         //Indica si esta habilitada la comunicación
bool inicio = true;               //Indica si es la primera vez que se ejecuta el código
bool antenas = false;             //Indica si las antenas están desplegadas
bool comandoValido = false;       //Indica si un comando recibido es valido
bool pausaTemperatura = false;    //Indica si se debe hacer una pausa debido a altas temperaturas
bool encriptacionActivada = true; //Indica si la encriptación esta activada
bool estadoPausa = false;         //Indica si se ejecuto el comando pausa
bool enEspera = false;            //Indica si se esta esperando algún comando
bool estadoPeriodo = false;       //Indica si se ejecuto el comando periodo

//Variables para guardar el paso del tiempo 
unsigned long tiempoAnteriorTransmision = 0;
unsigned long tiempoAnteriorTemperatura = 0;
unsigned long tiempoAnteriorCorriente = 0;
unsigned long tiempoPausaTemperatura = 0;
unsigned long tiempoNuevo = 0;
unsigned long tiempoPausa = 0;

void enviarDatosDiagnosticos(){
  //se envían los datos/medidas de los diferentes sensores
  
  wdt_reset(); //Se reinicia el contador del watchdog para evitar reinicios durante la transmisión
  
  //Primero se envían los valores de los sensores de Temperatura
  buff = malloc(50);
  sprintf(buff, "Temp: CPU=%sC, Batt1=%sC, Batt2=%sC, RFTR=%sC", temperaturaCPU, temperaturaBATT1, temperaturaBATT2, temperaturaRFTR);
  tamanioBuffer = strlen(buff);
  radio.sendWithRetry(RECEPTOR, buff, tamanioBuffer); //envía el paquete de datos al receptor sin esperar una respuesta
  free(buff);
  delay(3); //se esperan 3ms como es recomendado en la librería 
  
  //Luego se envían los valores de los sensores de corriente
  buff = malloc(50);
  sprintf(buff, "I5v=%s, I3.3v=%sA Prom I5v=%sA Prom I3v=%sA", corriente5v, corriente3v, corriente5vprom, corriente3vprom);
  tamanioBuffer = strlen(buff);
  radio.sendWithRetry(RECEPTOR, buff, tamanioBuffer);
  free(buff);
  delay(3); 

  //Y se envía el tiempo que ha pasado desde que se ha prendido/reiniciado
  buff = malloc(50);
  double ttemp = millis()/1000;
  sprintf(buff, "Tiempo activo: %f ",ttemp);
  tamanioBuffer = strlen(buff);
  radio.sendWithRetry(RECEPTOR, buff, tamanioBuffer);
  free(buff);
  delay(3); 
}

void esperaComando(){ //Recepción de comandos
  wdt_reset();
  if (radio.receiveDone()) { 
    
    char buffrec[radio.DATALEN];
    strcpy(buffrec,(char *)radio.DATA);
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }
    free(buff);
    buff = malloc(50);
    comandos = comandos + 1;
    
    if(enEspera == false){ 

      if(strcmp(buffrec,"BFC-Temperatura CPU") == 0){          //Comando para enviar datos del sensor de temperatura del microprocesador
        comandoValido = true;
        comandosTemperatura++;
        sprintf(buff, "Temperatura CPU = %sC", temperaturaCPU);
      }
      
      if(strcmp(buffrec,"BFC-Temperatura Batt1") == 0){        //Comando para enviar datos del sensor de temperatura de la batería 1
        comandoValido = true;
        comandosTemperatura++;
        sprintf(buff, "Temperatura Bateria 1 = %sC", temperaturaBATT1);
      }
      
      if(strcmp(buffrec,"BFC-Temperatura Batt2") == 0){       //Comando para enviar datos del sensor de temperatura de la batería 2
        comandoValido = true;
        comandosTemperatura++;
        sprintf(buff, "Temperatura Bateria 2 = %sC", temperaturaBATT2);
      }
      
      if(strcmp(buffrec,"BFC-Temperatura RFTR") == 0){          //Comando para enviar datos del sensor de temperatura del modulo de comunicaciones
        comandoValido = true;
        comandosTemperatura++;
        sprintf(buff, "Temperatura Transreciver = %sC", temperaturaRFTR);
      }
          
      if(strcmp(buffrec,"BFC-Consumo") == 0){                   //Comando para enviar datos de consumo de corriente
        comandoValido = true;
        comandosConsumo++;
        sprintf(buff, "I en Lineas 5V = %sA; Lineas 3.3V = %sA", corriente5vprom, corriente3vprom);    
      } 
      
      if(strcmp(buffrec,"BFC-Pausa") == 0){                  //Comando para pausar las comunicaciones por un tiempo en minutos
        comandoValido = true;
        if (encriptacionActivada == true){
          comandosPausa = comandosPausa + 1;
          sprintf(buff, "Ingrese tiempo en minutos");
          enEspera = true;
          estadoPausa = true;
        }else {
          sprintf(buff, "Primero debe activar la encriptacion de la red");
        }
      }
      
      if(strcmp(buffrec,"BFC-Periodo") == 0){                //Comando para cambiar el periodo de tiempo entre mensajes
        comandoValido = true;
        if (encriptacionActivada == true){
          comandosPausa = comandosPausa + 1;
          sprintf(buff, "Ingrese tiempo en milisegundos");
          enEspera = true;
          estadoPeriodo = true;
        }else {
          sprintf(buff, "Primero debe activar la encriptacion de la red");
        }
      }
      
      if(strcmp(buffrec,"BFC-Historial") == 0){            //Comando para ver que comandos se han ejecutado antes
        comandoValido = true;
        if (encriptacionActivada == true){
          sprintf(buff, "%d Comandos: T = %d; I: %d, P: %d, NV: %d", comandos, comandosTemperatura, comandosConsumo, comandosPausa, comandosNoValidos);
        }else {
          sprintf(buff, "Primero debe activar la encriptacion de la red");
        }
      }

      if(strcmp(buffrec,"BFC-DesactivarEnc") == 0){        //Comando para desactivar la encriptacion de los datos
        comandoValido = true;
        if (encriptacionActivada == true){
          radio.encrypt(null);
          encriptacionActivada = false; 
          sprintf(buff, "Encriptacion de datos desactivada");
        }else{
          sprintf(buff, "La encriptacion de datos ya esta desactivada");
        }
      }

      if(strcmp(buffrec,"BFC-ActivarEnc") == 0){          //Comando para activar la encriptacion de los datos
        comandoValido = true;
        if (encriptacionActivada == true){
          sprintf(buff, "Encriptacion de datos ya esta activada");
        }else{
          radio.encrypt(CLAVE);
          encriptacionActivada = true;
          sprintf(buff, "Encriptacion de datos activada");
        }
      }
      
      if(comandoValido == false){
        comandosNoValidosConsecutivos = comandosNoValidosConsecutivos + 1;
        comandosNoValidos = comandosNoValidos + 1;
        if(strlen(buffrec) < 30 ){
          sprintf(buff, "Comando no valido: %s", buffrec);
        }else{
          sprintf(buff, "Comando no valido"); 
        }
      }else{
        comandosNoValidosConsecutivos = 0;
      }

    }else{
      if(estadoPausa == true){ //Selección de tiempo para pausa de comunicaciones
        pausa = strtoul(buffrec,null,10);
        if (pausa >= 1 && pausa <=1440){
          sprintf(buff, "Se pausara todo proceso durante %d minutos", pausa);  
          pausa = pausa * 60 * 1000;
          tiempoPausa = millis();
        } else {
          sprintf(buff, "No es un valor valido o se supera las 24 horas");
          pausa = 0;
        }
        estadoPausa = false;
        comunicacion = false;
      }
       
      if(estadoPeriodo == true){ //Selección de periodo de envío de señales
        unsigned long periodoTransmisionTemp = strtoul(buffrec,null,10);
        if (periodoTransmisionTemp >= 1000 && periodoTransmisionTemp <=720000){            //entre 1 segundo y 12 horas
          periodoTransmision = periodoTransmisionTemp;
          sprintf(buff, "El periodo ahora es de %d milisegundos", periodoTransmision);  
        } else {
          sprintf(buff, "No es un valor valido o se superan las 12 horas");
        }
        estadoPeriodo = false;
      }
      enEspera = false;
    }
    tamanioBuffer = strlen(buff);
    radio.sendWithRetry(RECEPTOR, buff, tamanioBuffer);
    free(buff);
  }
}

void loop() {
  tiempoNuevo = millis();

  if (inicio == true && pausaTemperatura == false){ //Si es la primera vez que se ejecuta el código, se espera un tiempo indicado, se calienta la resistencia que despliega las antenas y luego se cambia la variable inicio para no volver a entrar en este estado, amenos que se detecte altas temperaturas
    wdt_reset(); //Se resetea el contador del watchdog, para evitar reinicios
    if((tiempoNuevo >= TIEMPOESPERA)&& (antenas == false)){
      antenas = true;
      digitalWrite(3, LOW);  
    }
    if(tiempoNuevo >= (TIEMPOESPERA + TIEMPOESPERARESISTENCIA)){
      digitalWrite(3, HIGH);
      inicio = false;  
    }
  }
  if(inicio == false && pausaTemperatura == false && comunicacion == true){ //Una vez que se ha esperado el tiempo indicado y se han desplegado las antenas se empieza un bucle donde se leen los diferentes sensores y se envían esos valores, o se recibe y contesta a comandos

    //Si no se han detectado demasiados fallos consecutivos al recibir comandos, se reinicia el watchdog
    if (comandosNoValidosConsecutivos <= NCOMANDOSNOVALIDOSANTESDEREINICIO){
       wdt_reset();
    }
    
    if (enEspera == false){  //si no se está esperando alguna comunicación
      
      //Se activan los sensores, se mide la temperatura, se guarda el valor y se vuelve a poner los sensores en estado de ahorro de energía (se pasa de un consumo de alrededor de 10uA a un consumo de alrededor de 0.5uA)
      if(tiempoNuevo - tiempoAnteriorTemperatura >= PERIODOMEDICIONTEMPERATURA){
        tiempoAnteriorTemperatura = tiempoNuevo;
        
        temperaturaCPU = malloc(6);
        temperaturaBATT1 = malloc(6);
        temperaturaBATT2 = malloc(6);
        temperaturaRFTR = malloc(6);
        
        sensorCPU.wakeup();                                       //se lee el sensor de temperatura TMP102 ubicado en la pcb del microcontrolador
        dtostrf(sensorCPU.readTempC(),5,2,temperaturaCPU);
        sensorCPU.sleep();
        
        sensorBATT1.wakeup();
        dtostrf(sensorBATT1.readTempC(),5,2,temperaturaBATT1);   //se lee el sensor de temperatura TMP102 ubicado debajo de la batería 1
        sensorBATT1.sleep();
      
        sensorBATT2.wakeup();
        dtostrf(sensorBATT2.readTempC(),5,2,temperaturaBATT2);   //se lee el sensor de temperatura TMP102 ubicado debajo de la batería 2
        sensorBATT2.sleep();
        
        dtostrf(radio.readTemperature(-1),5,2,temperaturaRFTR); //se lee el sensor de temperatura incluido en el RFM69HCW
      }
  
      //Se lee los valores de corriente medidos por el sensor de corriente ACS712 en las lineas de 5V y 3.3V 
        
      if(tiempoNuevo - tiempoAnteriorCorriente >= PERIODOMEDICIONCORRIENTE){
        tiempoAnteriorCorriente = tiempoNuevo;
        
        corriente3v = malloc(7);
        corriente5v = malloc(7);
        corriente3vprom = malloc(7);
        corriente5vprom = malloc(7);
        
        i5 = ( analogRead(A0)*(4.98/1024.0) - 2.5 )/SENSIBILIDADSENSORCORRIENTE5V;
        i5 = i5 < 0? i5 * -1 : i5; 
        fabs(i5);
        i5prom = (i5prom + i5)/2;
        dtostrf(i5,6,4,corriente5v);
        dtostrf(i5prom,6,4,corriente5vprom);
        
        i3 = ( analogRead(A1)*(4.98/1024.0) - 2.5 )/SENSIBILIDADSENSORCORRIENTE3V;
        i3 = i3 < 0? i3 * -1 : i3;
        fabs(i3);
        i3prom = (i3prom + i3)/2;
        dtostrf(i3,6,4,corriente3v);
        dtostrf(i3prom,6,4,corriente3vprom); 
      }
      
      //Se envían datos diagnósticos del cubesat
      if(tiempoNuevo - tiempoAnteriorTransmision >= periodoTransmision){
        tiempoAnteriorTransmision = tiempoNuevo;
        enviarDatosDiagnosticos();
      }
    }
    
    esperaComando();
    
    free(temperaturaCPU);
    free(temperaturaBATT1);
    free(temperaturaBATT2);
    free(temperaturaRFTR);
    free(corriente3v);
    free(corriente5v);
    free(corriente3vprom);
    free(corriente5vprom);
  }
  if (comunicacion == false && pausaTemperatura == false){
     if(tiempoNuevo - tiempoPausa > pausa){
        pausa = 0;
        comunicacion = true;
     }
  }
  
  //Si se ha detectado temperaturas peligrosas detiene el envío de datos y la lectura de los sensores 
  if(digitalRead(PIN_ALERTA_TEMP_CPU) == 1 ||  digitalRead(PIN_ALERTA_TEMP_BATT1) == 1 ||  digitalRead(PIN_ALERTA_TEMP_BATT2) == 1){
    pausaTemperatura =  true;
    tiempoPausaTemperatura = millis();
    if (inicio == true){
      digitalWrite(3, HIGH);
    }
  }
  
  //Si se ha detectado altas temperaturas, se deja que pase un tiempo previamente indicado antes de desactivar la pausa  
  if( pausaTemperatura == true ){
    wdt_reset();
    if (tiempoNuevo - tiempoPausaTemperatura >= TIEMPOESPERAALERTATEMPERATURA){
      pausaTemperatura = false;
    }
  }
}
