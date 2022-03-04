/*********************************************************************
  PROYECTO BALAM - PRIMER ENCUENTRO NACIONAL DE ROBÓTICA 2018

  ESTE ES UN CÓDIGO BASE

  POR FAVOR NO MODIFIQUE ESTE CÓDIGO
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



/*
   DECLARAMOS VARIABLES PARA NUESTRO SHIELD DE MOTORES | PUEDE MODIFICAR ESTE CÓDIGO
*/

// CREAMOS LA INSTANCIA DEL SHIELD DE MOTORES
Adafruit_MotorShield FEATHERWING = Adafruit_MotorShield();

// And connect 2 DC motors to port M1 & M4 !
Adafruit_DCMotor *I_MOTOR = FEATHERWING.getMotor(1);
Adafruit_DCMotor *D_MOTOR = FEATHERWING.getMotor(3);

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif


// DECLARAMOS VARIABLES DE VELOCIDAD - VALOR MÁXIMO 255
#define VelocidadAdelante             200
#define VelocidadAtras                100
#define VelocidadGiro                 100


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



/*=========================================================================
  CÓDIGO PARA CONFIGURACIÓN DE BLUETOOTH LOW ENERGY
    -----------------------------------------------------------------------*/


#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/






//COLORCAR NOMBRE A NUESTRO CARRO


String BROADCAST_NAME = "ROBOMATRIX";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


//PEQUEÑA AYUDA
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


//AGREGAMOS BUFFER PARA NOMBRE
char buf[60];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=


void setup(void)
{

  myservo.attach(9);

  FEATHERWING.begin();  // create with the default frequency 1.6KHz

  // APAGAMOS MOTORES
  I_MOTOR->setSpeed(0);
  I_MOTOR->run(RELEASE);

  D_MOTOR->setSpeed(0);
  D_MOTOR->run(RELEASE);


  // NO MODIFICAR ESTE CÓDIGO

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //Convertimos el nombre a comando char
  BROADCAST_CMD.toCharArray(buf, 60);

  //Change the broadcast device name here!
  if (ble.sendCommandCheckOK(buf)) {
    Serial.println("name changed");
  }
  delay(250);

  //reset to take effect
  if (ble.sendCommandCheckOK("ATZ")) {
    Serial.println("resetting");
  }
  delay(250);

  //Confirm name change
  ble.sendCommandCheckOK("AT+GAPDEVNAME");

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}


/*
   DECLARAMOS NUESTRAS FUNCIONES DE MOVIMIENTO
*/

void moverAdelante() {
  I_MOTOR->setSpeed(VelocidadAdelante);
  I_MOTOR->run(FORWARD);

  D_MOTOR->setSpeed(VelocidadAdelante);
  D_MOTOR->run(FORWARD);
}

void moverAtras() {

  // se hace este pequeño hack de detener los motores durante 300 milisegundos para que la reversa no sea tan repentina.
  I_MOTOR->run(RELEASE);
  D_MOTOR->run(RELEASE);
  delay(300);

  I_MOTOR->setSpeed(VelocidadAtras);
  I_MOTOR->run(BACKWARD);

  D_MOTOR->setSpeed(VelocidadAtras);
  D_MOTOR->run(BACKWARD);
}


void girarDerecha() {
  I_MOTOR->setSpeed(VelocidadGiro);
  I_MOTOR->run(FORWARD);

  D_MOTOR->setSpeed(0);
  D_MOTOR->run(RELEASE);

}

void girarIzquierda() {
  D_MOTOR->setSpeed(VelocidadGiro);
  D_MOTOR->run(FORWARD);

  I_MOTOR->setSpeed(0);
  I_MOTOR->run(RELEASE);

}
void detener() {
  I_MOTOR->setSpeed(0);
  I_MOTOR->run(RELEASE);

  D_MOTOR->setSpeed(0);
  D_MOTOR->run(RELEASE);
}





/**************************************************************************/
/*!
    FUNCIÓN LOOP = DONDE VA NUESTRO CÓDIGO
*/
/**************************************************************************/
void loop(void)
{

  /*
     NO MODIFICAR ESTE BLOQUE DE CÓDIGO
  */

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" PRESIONADO");


      /*
         ESTE BLOQUE DE CÓDIGO ES DONDE CONFIGURAMOS CADA BOTON DE NUESTRO CONTROL PAD
         Y ASIGNAMOS QUE ACCIÓN TENDRÁ NUESTROS BOTONES | SI SE DESEA AGREGAR UN BOTÓN NUEVO SOLO SE AGREGARÁ UN IF VERIFICANDO QUE SE PRESIONE ESE BOTÓN
      */
      if (buttnum == 5) {
        moverAdelante();
        ble.println("ADELANTE");
      }

      if (buttnum == 6) {
        moverAtras();
        ble.println("ATRAS");
      }

      if (buttnum == 7) {
        girarDerecha();
        ble.println("IZQUIERDA");
      }


      if (buttnum == 8) {
        girarIzquierda();
        ble.println("DERECHA");
      }
      if (buttnum == 1) {
        detener();
        ble.println("DETENER");
      }
      if (buttnum == 2) {
        for (pos = 80; pos <= 180; pos += 1) {
          myservo.write(pos);
          delay(15);
        }
      }
      if (buttnum == 4) {
        for (pos = 180; pos >= 80; pos -= 1) {
          myservo.write(pos);
          delay(15);
        }
      }


    } else {
      Serial.println("NO PRESIONADO");
    }

  }

}
