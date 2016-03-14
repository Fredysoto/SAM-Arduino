//Adquisicion de datos para proyecto SAM Arduino
//recopila datos de los sensores de aceleracion, presion, humedad y temperatura y gps
//guarda datos en memoria micro SD, y envía los datos cuando se tenga señal
//detecta movimentos bruscos y lo indica poniendo un 1 al principio de la cadena de datos
//Guarda el estado de la cola y nombre de archivo actual en la memoria EEPROM por si se corta la energia
//Fredy Soto 2016

//::::::bibliotecas adquisicion de datos:::::::::
#include <Wire.h> //Biblioteca para I2C
#include <SFE_MMA8452Q.h> //Biblioteca para Acelerometro
#include <DHT.h> //Biblioteca para sensor de temperatura y Humedad
#include <SparkFunMPL3115A2.h> //biblioteca para sensor de presion
#include <Adafruit_GPS.h> // biblioteca de GPS
//::::::bibliotecas guardado y lectura SD:::::::::
#include <SD.h> //Biblioteca de tarjeta SD
#include <MemoryFree.h> //bibilioteca para tratamiento de memoria
//::::::bibliotecas Envio Xbee:::::::::
#include <XBee.h> //Biblioteca de Xbee para la comunicacion inalambrica (envio de datos)
//::::::biblioteca para guardar datos de manejo de colas en eeprom::::::
#include <EEPROM.h>


//::::Variables adquisicion de datos:::::
#define GPSECHO  false
#define DHTPIN              2    // pin a conectar
#define DHTTYPE           DHT22     //definicion de sensor de temperatura y humedad
boolean usingInterrupt = false ; //interrupcion para GPS
void useInterrupt(boolean); //interrupcion para GPS
char s[32]; //char para transformacion de string a char
int  ledPower = 22;  // pin para led de sensor PM
int dustPin = 1; // pin para lectura analoga de material particulado

//::::::variables guardado y lectura SD:::::::::
const int chipSelect = 8;  //CS para la tarjeta de memoria
int Reg_actual = 0; // Es el contador que contiene el numero actual de registros guardados en el archivo de texto.
#define Reg_total 50 //Contiene el numero total de registros (mediciones) a guardar en un archivo de texto .txt
int cola = 0; //contiene la posicion de la cola de envio
int num_archivo = 0; // Contiene el numero de archivos de texto
int espera = 1000;
char buf[157]; //buffer para la lectura de SD
//int  lengthbuf=158;

//::::::variables Envio Xbee:::::::::
uint8_t payload[159] = {};//dato para cargarle al xbee
int e = 0; //indicador de cualquier error en la transmision de datos
int ping = 0; //contador para enviar el test de señal para xbee
int Rssi;

//::::::direcciones EEPROM:::::::::
#define dir_archi 4 //direccion de la memoria EEPROM donde se almacena el numero de archivo
#define dir_stat 5 //indica si la eeprom se escribio anteriormente (1) o si se leyó anteriormente (0), cualquier otro valor significa que nunca se usó



//::::Objetos adquisicion de datos:::::
MMA8452Q  Acel; //creacion de instancia para acelerometro
DHT sht(DHTPIN, DHTTYPE); //creacion de instancia para sensor de temperatura y humedad
MPL3115A2 sp; //instancia para sensor de presion
HardwareSerial mySerial = Serial3; // habilita la comunicacion serial por el puerto 3
Adafruit_GPS GPS(&Serial3); //indica que la comunicacion serial para le gps se hara en el puerto 3
//::::::Objetos guardado y lectura SD:::::::::
File myFile; //biblioteca para el manejo de archivos en SD
//::::::objetos Envio Xbee:::::::::
XBee xbee = XBee(); //biblioteca para el envio de datos en Xbee
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x00000000); // indica la direccion a quien se le enviara el paquete api
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload)); //
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

void setup() {
  pinMode(3, INPUT); //pin para la lectura de la disponibilidad de señal XBee (RSSI)
  pinMode(10, OUTPUT); //
  pinMode(ledPower,OUTPUT);// pin para el sensor de material particulado
  Serial.begin(57600); //inicializacion de comunicacion serial para lectura en pc 
  Serial.println("INICIO"); 
  GPS.begin(9600); //inicializacion de la comunicacion serial con el GPS 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz actualizacion cada segundo
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  Acel.init(); //inicializacion de acelerometro
  sht.begin(); //inicializacion de sensor de humedad y temperatura
  sp.begin(); //inicializacion de sensor de presion
  Serial2.begin(57600); //inicializacion de comunicacion serial para Xbee
  xbee.setSerial(Serial2); //asignacion de puerto serial para xbee

if (!SD.begin(chipSelect)) { //indicador de buena inicializacion de la tarjeta SD
    Serial.println("initialization failed!");
    return;
  }
//configuracion de sensor de presion
  sp.setModeBarometer(); // indica la medicion de presion en Pascales desde 20 a 110 kPa
  sp.setOversampleRate(7); //Setea el sobremuestreo a 128
  sp.enableEventFlags(); // habilita las alertas de presion y temperatura

  switch (EEPROM.read(dir_stat)) {
    case 1 :
      Read_Status_EEPROM();
      Serial.println("cargando EEPROM escrita");
      break;
    case 0 :
      Read_Status_EEPROM();
      Serial.println("cargando EEPROM leida");
      break;
    default:      num_archivo = 0; Save_Status_EEPROM();
      Serial.println("reiniciando EEPROM");
      break;
  }
}


void loop() {
  //::::::::envio prueba de señal Xbee:::::::
  if (ping == 5) {
    if (Rssi != 1) {
      envio("test"); //envio de prueba para ver si recibimos respuesta y rssi
    }
    ping = 0;
  }
  ping++;

  //:::::::::CONFIGURACIONES GPS::::::::::::::
  if (GPS.newNMEAreceived()) { //si se recibio una nueva lectura del GPS
    if (!GPS.parse(GPS.lastNMEA()))   // esto tambien setea el flag newNMEAreceived() en falso
      return;  // en caso de fallar en el ordenamiento de los datos, se espera por otro
  }

  //:::::lectura de datos y desicion de envio o guardado:::::::::
  Rssi = digitalRead(3); //lectura de pin Rssi de xbee
  String frame = readSensors();  //lectura de datos de sensores
  Serial.println(frame);
      if (Rssi >= 1) {

      if (cola < num_archivo) {
        Serial.print("leyendo y enviando....  ");
        Serial.print(cola);
        Serial.print(" de ");
        Serial.println(num_archivo);
        readSD(); //lee los datos de la memoria y los envia mediante xbee
        envio(frame);// envia el dato leido en el loop luego de enviar todos los datos de un archivo para no perderlo.
      }  else {
        cola = 0;
        num_archivo = 0;
        Save_Status_EEPROM();
        envio(frame);
        Serial.println("enviando....");
        delay(espera);
      }
    } else {
      saveSD(frame);
      Serial.print("guardando archivo numero :");
      Serial.println(num_archivo);
      delay(espera);
    }
  Serial.print("cola ");
  Serial.print(cola);
  Serial.print("  archi ");
  Serial.println(num_archivo);
        Serial.print("\n");
}


String readSensors() { //Funcion para la lectura de todos los sensores y creacion del frame
  Acel.read();
  String frame = "";
  frame += "{\"Tap\" : ";
  if (Acel.readTap() != 0) { //lee la indicacion de movimiento por sobre 2G de acelerometro
    frame += 1;
  } else {
    frame += 0;
  }
  frame += ", \"Presion\": ";
  if(sp.readPressure()<0){ //recibe la medicion de presion
    frame +="     0";
  }else{
  frame += dtostrf(sp.readPressure(), 6, 0, s);
  }
  frame += ", \"Temperatura\": ";
  frame += dtostrf(sht.readTemperature(), 4, 1, s); //lee la temperatura
  frame += ", \"Humedad\": ";
  frame += dtostrf(sht.readHumidity(), 4, 1, s); //lectura de la humedad
  frame += ", \"Sharp\": ";
  frame += dtostrf(dust(), 9, 4, s); //lectura del material particulado
  frame += ", \"Gps\" :{";
  frame += "\"lat\": ";
  if(GPS.latitudeDegrees !=0){ //lectura de la latitud en grados
   frame += dtostrf(GPS.latitudeDegrees, 7, 4, s);
  }else{
    frame +="  0.0000";
  }
  frame += ", \"lon\": ";
  if(GPS.longitudeDegrees !=0){  //lectura de la longitud en grados
  frame += dtostrf(GPS.longitudeDegrees, 7, 4, s);
  }else{
    frame +="  0.0000";
  }
  frame += ", \"time\": ";  //Lectura del año, mes, dia, hora, minutos y segundos

  if (GPS.year < 10)  {
    frame += 20;
    frame += 0;
    frame += GPS.year;
  }
  else {
    frame += 20;
    frame += String(GPS.year);
  }
  if (GPS.month < 10)  {
    frame += 0;
    frame += GPS.month;
  }
  else {
    frame += String(GPS.month);
  }
  if (GPS.day < 10)  {
    frame += 0;
    frame += GPS.day;
  }
  else {
    frame += String(GPS.day);
  }

  if (GPS.hour< 10)  {
    frame += 0;
    frame += GPS.hour;
  }
  else {
    frame += String(GPS.hour);
  }
  if (GPS.minute < 10)  {
    frame += 0;
    frame += GPS.minute;
  }
  else {
    frame += String(GPS.minute);
  }
  if (GPS.seconds < 10)  {
    frame += 0;
    frame += GPS.seconds;
  }
  else {
    frame += String(GPS.seconds);
  }
  frame += "}}";
  return frame;
}

void saveSD(String frame) { //funcion para guardado del frame en la tarjeta SD
  if (Reg_actual < Reg_total) {
    char * file_nameSave = nameSD(num_archivo); //entrega el nombre del archivo segun el numero de cola general
    writeSD(file_nameSave, frame); //escribe el frame en la tarjeta SD dentro del archivo con el nombre indicado
    Reg_actual++;
    free(file_nameSave);
  }
  else {
    num_archivo++; //si se sobrepasa la cantidad de registros asignados a cada archivo, se aumenta el numero del archivo
    char * file_nameSave = nameSD(num_archivo); //entrega el nombre del archivo segun el numero de cola general
    writeSD(file_nameSave, frame);
    free(file_nameSave); //se borra de la memoria el char creado
    Reg_actual = 1;
    Save_Status_EEPROM(); //se guarda el numero de archivo en la memoria EEPROM 
  }
}

char *nameSD(int num_txt) { //devuelve un char con el nombre del archivo formateado para la tarjeta SD
  String file = String(num_txt, DEC) + ".TXT";
  char *Fname = new char [file.length() + 1];// el problema esta en la creacion de un nuevo char siempre
  strcpy(Fname, file.c_str());
  return Fname;
}

void writeSD(const char* filename, String content) { //escribe el contenido en la tarjeta SD segun el nombre del archivo
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.println(content);
    myFile.close();
  } else {
    Serial.println("no se pudo guardar");
  }
  return;
}

void Save_Status_EEPROM() { //guarda el estatus de la cola en la memoria EEPROM
  EEPROM.write(dir_archi, num_archivo);//Guardado Numero archivo para escritura SD
  EEPROM.write(dir_stat, 1);
}


void Read_Status_EEPROM() { //lee el estatus de la cola desde la memoria EEPROM
  num_archivo = EEPROM.read(dir_archi);
  EEPROM.write(dir_stat, 0);
}


void readSD() { //funcion para la lectura de los frames desde la tarjeta SD y manejo de cola de envio
  int count = 0; //contador para limitar el envio de datos fallidos mas abajo
  char * file_nameRead = nameSD(cola); //entrega el nombre del archivo segun el numero de cola general
  myFile = SD.open(file_nameRead);
  
  if (myFile) {
    while (myFile.position() < myFile.size()) {
               
      myFile.read(buf, sizeof(buf)); //lee lo que esta en la sd y lo escribe en el puerto serial
      e = 0;
      envio(buf);
      Serial.print("posicion ");
      Serial.print(myFile.position());
      Serial.print(" de ");
      Serial.println(myFile.size());

      while (e == 1 && digitalRead(3) == 1) { //entra al ciclo si existe un error de envio y hay señal
        envio(buf);
        if (digitalRead(3) != 1) {
          e = 0;
          Serial.println("no hay conexion");
        }
        count++;
        if (count > 2) break; //libera el while si es que han habido 2 intentos fallidos
      }
    }
    myFile.close(); // close the file:
    SD.remove(file_nameRead); //borra el archivo enviado no probado
    cola++;
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening: ");
    Serial.println(file_nameRead);
    cola++;
  }
  free(file_nameRead);
  return;
}

void envio(String data) {// envia el string que se le pase mediante el XBee
  int l = data.length() + 1;
  char * y = new char [l];
  strcpy(y, data.c_str());
  for (int i = 0; i < l; i++) {
    payload[i] = y[i];
  }
  xbee.send(zbTx);
  if (xbee.readPacket(500)) {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getDeliveryStatus() == SUCCESS) {
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("el xbee remoto no recibio el paquete");
        e = 1;
      }
    }
  } else if (xbee.getResponse().isError()) {
    Serial.println("error leyendo el paquete");
    e = 1;
  } else {
    Serial.println("error: el xbee local no dio el status de transmision a tiempo");
    e = 1;
  }
  free(y);
  return ;
}

//interrupciones para GPS
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

//medicion de sensor SHARP GP2Y10
float dust(){ //realiza la medicion del material particulado segun las especificaicones del fabricante

    int delayTime=280;
    int delayTime2=40;
    float offTime=9680;
    
  int dustVal=0;
  float voltage = 0;
 
digitalWrite(ledPower,LOW); // enciende el led
  delayMicroseconds(delayTime);
  
  dustVal=analogRead(dustPin); // lee el valor del material particulado, demora cerca de 100 uSegundos
  
  delayMicroseconds(delayTime2);
  digitalWrite(ledPower,HIGH); //Apaga el led
  delayMicroseconds(offTime);

  voltage = dustVal*(5.0/1024); //conversion de cuentas a voltaje
return voltage;
  }


