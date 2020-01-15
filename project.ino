#include <Arduino_FreeRTOS.h>

void print_string_serial(char *string){
  Serial.print(string);
  Serial.flush();
}

void print_number_serial(int number){
  Serial.print(number);
  Serial.flush();
}

void print_float_serial(float number){
  Serial.print(number);
  Serial.flush();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");
  Serial.flush();
  set_print_str(&print_string_serial);
  set_print_num(&print_number_serial);
  set_print_float(&print_float_serial);
  xTaskCreatePeriodic(reader, "", 120, "", 2, NULL, 0, 0, 0);

}


void reader(){
char input[80];
  
  int i;
  while(1){
    if(Serial.available()){
      i = 0;
      while(Serial.available()){
        input[i++] = Serial.read();
      }
      input[i] = 0;    
      parseInput(input);
    }
    vTaskDelay(10);
  }
}

void loop() {}