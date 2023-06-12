#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(49, 48); // CE, CSN
const byte addresses [][6] = {"00001", "00002"};  //Setting the two addresses. One for transmitting and one for receiving


//----------reading address ---------------//

// 00001 - for secondary robot reading address
// 00002 - for primary robot reading address

//----------------------------------------//
void setup() {
  Serial.begin(9600);
  radio.begin();                           //Starting the radio communication
  radio.openWritingPipe(addresses[0]);     //Setting the address at which we will send the data
  radio.openReadingPipe(0, addresses[1]);  //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);           //You can set it as minimum or maximum depending on the distance between                                                the transmitter and receiver. 
}

//for sending a word
void sending(const char* message) {
    delay(5);
  
    // Stop receiving and set the module as transmitter
    radio.stopListening();     

    // Send the message five times to reduce errors
    for(int i = 0; i < 5; i++) {
        radio.write(message, strlen(message) + 1);
        delay(20);
    }
}

//outputing the reciving word
String receiving(){
  
  delay(5);
  
  radio.startListening();   //This sets the module as transmitter     

  //reading the incomming massage  
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    //Serial.println(text);
    return String(text);
  }

  return "";

}

//testing function...
/////////////////////////////////////////////////////////////


int i=0;
String receivedText;


void loop() 

{ 
  
if (i==0){

String receivedText = receiving();
Serial.println("Received text: " + receivedText);
  if (receivedText=="red"){
    sending("ok");
    i=1;
  }
}


}