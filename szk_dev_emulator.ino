#include <Arduino.h>
#include "wiring_private.h"
#include <Chrono.h>

extern "C"{
#include "bkkproto.h"  
}

#define SERIAL_DBK_SPEED 115200L
//Скорость порта 1
#define PORT_1_SPEED 19200L
//Скорость порта 2
#define PORT_2_SPEED SERIAL_DBK_SPEED
//Скорость порта 3
#define PORT_3_SPEED SERIAL_DBK_SPEED

#define Logp Serial
#define PORT_TRANSMIT HIGH
#define PORT_RECEIVE  LOW
#define SERIAL_PORT_TRANSMIT(_pin_)  do{ digitalWrite(_pin_, PORT_TRANSMIT);delayMicroseconds(50);}while(0);
#define SERIAL_PORT_RECEIVE(_pin_)   digitalWrite(_pin_, PORT_RECEIVE)


#define ETHERNET_CS_PIN   A3
#define SDCARD_CS_PIN     A2

//Пин определяющий "HIGH" - мастер,  "LOW" - ведомый.
#define N_SLAVE_PIN 1
int is_master = 0;



#if defined(ARDUINO_SAMD_MKRZERO) || defined(ARDUINO_SAMD_MKR1000)
typedef Uart SerialKlass;

//RTCZero rtc;
#define WDT_MS      500
#define WDT_LOOP_MS 250
#define NO_RECV_CNTR_MAX          3




//1 Порт - штатный
#define SerialPort1         Serial1
#define SERIAL_PORT_1_SEND_PIN    A4
#define SERIAL_PORT_1_RX_PIN      13
#define SERIAL_PORT_1_TX_PIN      14

//#define INDIC_PIN LED_BUILTIN
#define INDIC_PIN LED_BUILTIN

//2 Порт - перепрограммируем


#define SERIAL_PORT_2_TX_PIN      4
#define SERIAL_PORT_2_RX_PIN      5

#define SERIAL_PORT_2_SEND_PIN    A5

Uart    SerialPort2(&sercom4, SERIAL_PORT_2_RX_PIN, SERIAL_PORT_2_TX_PIN, SERCOM_RX_PAD_3, UART_TX_PAD_2 );
extern "C" {
  void SERCOM4_Handler()  {
    SerialPort2.IrqHandler();
  }
}

//3 Порт - перепрограммируем
#define SERIAL_PORT_3_TX_PIN      6
#define SERIAL_PORT_3_RX_PIN      7
#define SERIAL_PORT_3_SEND_PIN    A6

Uart    SerialPort3(&sercom3, SERIAL_PORT_3_RX_PIN, SERIAL_PORT_3_TX_PIN, SERCOM_RX_PAD_3, UART_TX_PAD_2 );
extern "C" {
  void SERCOM3_Handler()  {
    SerialPort3.IrqHandler();
  }
}


#define N_PORTS 3
#define N_PORTS_BASE 3
#endif //MKR

struct PortDef {
  SerialKlass* port;
  byte  port_settings;
  int port_timeout; //msec
  int send_pin_no;
  int n_rx;
  int n_tx;
  int n_rx_err;
  int n_incorr;
  int inpack_n_devs;
  int ix_dev;
  int current_dev;
  int out_current_dev;
  int inpack_data_pos;
  int outpack_data_pos;
  int state; // 0 - stop, 1- send, 2 - read
  int empty_tx_size;
  Chrono* read_tm;
};

#define DEFAULT_PORT_SETTINGS (SPD38400|RS_SERIAL_8E1)
#define DEFAULT_PORT_TIMEOUT 0
#define WRITE_CHANK_SIZE  20

struct PortDef rs_ports[N_PORTS_BASE] = {
  { .port = &SerialPort1,
    .port_settings = DEFAULT_PORT_SETTINGS,
    .port_timeout = DEFAULT_PORT_TIMEOUT,
    .send_pin_no = SERIAL_PORT_1_SEND_PIN ,
    .n_rx = 0,
    .n_tx = 0,
    .n_rx_err = 0,
    .n_incorr = 0,
    .inpack_n_devs = 0,
    .ix_dev = 0,
    .current_dev = 0,
    .out_current_dev = 0,
    .inpack_data_pos = 0,
    .outpack_data_pos = 0,
    .state = 0,
    .empty_tx_size = 0,
  },
  { .port = &SerialPort2,
    .port_settings = DEFAULT_PORT_SETTINGS,
    .port_timeout = DEFAULT_PORT_TIMEOUT,
    .send_pin_no = SERIAL_PORT_2_SEND_PIN ,
    .n_rx = 0,
    .n_tx = 0,
    .n_rx_err = 0,
    .n_incorr = 0,
    .inpack_n_devs = 0,
    .ix_dev = 0,
    .current_dev = 0,
    .out_current_dev = 0,
    .inpack_data_pos = 0,
    .outpack_data_pos = 0,
    .state = 0,
    .empty_tx_size = 0,
  },
  { .port = &SerialPort3,
    .port_settings = DEFAULT_PORT_SETTINGS,
    .port_timeout = DEFAULT_PORT_TIMEOUT,
    .send_pin_no = SERIAL_PORT_3_SEND_PIN ,
    .n_rx = 0,
    .n_tx = 0,
    .n_rx_err = 0,
    .n_incorr = 0,
    .inpack_n_devs = 0,
    .ix_dev = 0,
    .current_dev = 0,
    .out_current_dev = 0,
    .inpack_data_pos = 0,
    .outpack_data_pos = 0,
    .state = 0
  }
};

String st_mkr_name("[Device Imitator]");
Chrono diag_poll;


void setup() {
  Logp.begin(115200);
  setup_board();
}

void loop() {
  serial_job();
  if (diag_poll.hasPassed(500)) {
    diag_poll.restart();
    diaglog();
  }
}



void setup_board() {
  pinMode(INDIC_PIN, OUTPUT);
  pinMode(SDCARD_CS_PIN, OUTPUT);
  digitalWrite(SDCARD_CS_PIN, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  indic(0);
  pinMode(N_SLAVE_PIN, INPUT_PULLUP);
  is_master = 0;
  for (int i = 0 ; i < 3; i++) {
    delay(1);
    is_master += digitalRead(N_SLAVE_PIN) == LOW ? -1 : 1;
  }
  is_master = is_master > 0 ? 1 : 0;
  if (!is_master) {
    st_mkr_name = "[slave]";
  }
#if defined(ARDUINO_SAMD_MKRZERO) || defined(ARDUINO_SAMD_MKR1000)
  pinMode(SERIAL_PORT_1_RX_PIN, INPUT_PULLUP); 
  
  pinMode(SERIAL_PORT_2_RX_PIN, INPUT_PULLUP);
  pinPeripheral(SERIAL_PORT_2_TX_PIN, PIO_SERCOM_ALT);
  pinPeripheral(SERIAL_PORT_2_RX_PIN, PIO_SERCOM_ALT);

  pinMode(SERIAL_PORT_3_RX_PIN, INPUT_PULLUP);
  pinPeripheral(SERIAL_PORT_3_TX_PIN, PIO_SERCOM_ALT);
  pinPeripheral(SERIAL_PORT_3_RX_PIN, PIO_SERCOM_ALT);

  //pinMode(SERIAL_PORT_3_RX_PIN, INPUT_PULLUP);
#endif //MKR Special

  SerialPort1.begin(PORT_1_SPEED, SERIAL_8N2);
  SerialPort2.begin(PORT_2_SPEED, SERIAL_8N1);
  SerialPort3.begin(PORT_3_SPEED, SERIAL_8N1);

  //rs_ports[0] = {.port = &SerialPort1, .send_pin_no = SERIAL_PORT_1_SEND_PIN, };
  //rs_ports[1] = {.port = &SerialPort2, .send_pin_no = SERIAL_PORT_2_SEND_PIN  };
  //rs_ports[2] = {.port = &SerialPort3, .send_pin_no = SERIAL_PORT_3_SEND_PIN  };

#if defined(ARDUINO_TEENSY35) || defined (ARDUINO_TEENSY36)
  SerialPort4.begin(PORT_4_SPEED, SERIAL_8N1);
  rs_ports[3].port = &SerialPort4;
  rs_ports[3].send_pin_no = SERIAL_PORT_4_SEND_PIN;
#endif //TEENSY

  for ( int i = 0; i < N_PORTS; i++) {
    pinMode(rs_ports[i].send_pin_no, OUTPUT);
    SERIAL_PORT_RECEIVE(rs_ports[i].send_pin_no);
    rs_ports[i].read_tm = new Chrono( millis, false); //startNow = false
    rs_ports[i].port->setTimeout(0);
  }
  Logp.print(st_mkr_name);
  Logp.println("... setup Ok.");
}



void indic(int state) {
  if (state) {
    digitalWrite(INDIC_PIN, HIGH);
  } else {
    digitalWrite(INDIC_PIN, LOW);
  }
}

byte buff[18] = {0x00,0x01,0xae,0x30,0x00,0x00,0xa1,0xb2,0xc3,0xd4,0xe5,0xf6,0x00,0x00,0x00,0x00,0xcc,0xdd};
byte inbuff[N_PORTS_BASE][sizeof(buff)];
const static int buff_len = sizeof(buff);
const static int n_devs = 30;

byte rio_req_buff[] = {"$206"};
const static int rio_req_buff_len = sizeof(rio_req_buff) - 1;
byte rio_ack_buff[] = {"!000f00\n"};
const static int rio_ack_buff_len = sizeof(rio_ack_buff) - 1;
byte rio_tu_buff[] = {"#20"};
const static int rio_tu_buff_len = sizeof(rio_tu_buff) - 1;
byte rio_ack_tu_buff[] = {">\n"};
const static int rio_ack_tu_buff_len = sizeof(rio_ack_tu_buff) - 1;


void serial_job(){
  unsigned long ct = micros();
  unsigned long n_delay = (1000000L*10*buff_len*2*N_PORTS)/SERIAL_DBK_SPEED;
  unsigned long nt = ct+n_delay;
  int rx_pos[N_PORTS_BASE] = {0,0,0};
  int n_ok_recv = 0;

  unsigned long read_tm[N_PORTS] = {0,0,0};
  for(;;){
    int msk_ok = 0x0;
    for (int j = 0 ; j < N_PORTS; j++){
      msk_ok|= 0x1<<j;
      Stream* port = rs_ports[j].port;
      while(port->available() && rx_pos[j] < buff_len){
        byte c = port->read();  
        read_tm[j] = micros();
        if(rx_pos[j]==0){
          nt = micros()+n_delay;  
        }
        inbuff[j][rx_pos[j]] = c;
        rx_pos[j] += 1;
        ++rs_ports[j].n_rx;
      }
      
      if(rx_pos[j]==rio_req_buff_len ){
        if(!memcmp(inbuff[j],rio_req_buff,rio_req_buff_len)){
          indic(1);
          n_ok_recv|=0x1<<j;
          //answer echo
          SERIAL_PORT_TRANSMIT(rs_ports[j].send_pin_no);
          port->write(rio_ack_buff, rio_ack_buff_len);
          port->flush();
          SERIAL_PORT_RECEIVE(rs_ports[j].send_pin_no);
          rs_ports[j].n_tx += rio_ack_buff_len;
          rx_pos[j]=0;
          indic(0);
        }
      }else if (rx_pos[j]==rio_tu_buff_len ){
        if(!memcmp(inbuff[j],rio_tu_buff,rio_tu_buff_len)){
          indic(1);
          n_ok_recv|=0x1<<j;
          //answer echo
          SERIAL_PORT_TRANSMIT(rs_ports[j].send_pin_no);
          port->write(rio_ack_tu_buff, rio_ack_tu_buff_len);
          port->flush();
          SERIAL_PORT_RECEIVE(rs_ports[j].send_pin_no);
          rs_ports[j].n_tx += rio_ack_tu_buff_len;
          rx_pos[j]=0;
          indic(0);
        }
      }else{
        if( rx_pos[j] != 0 && (micros() - read_tm[j]) > 2000){
          rx_pos[j] = 0;
        }
          
      }
    }
    if (micros()>nt || n_ok_recv == msk_ok){
        
        break;  
    }
  }
}


void diaglog() {
  static int diag_poll_cntr = 0 ;
  //digitalWrite(LED_BUILTIN, HIGH);
  Logp.println();

  if (diag_poll_cntr == 0) {
    Logp.print("dcntr,Mkr,");
    for (int j = 0; j < N_PORTS; j++) {
      Logp.print("Port");
      Logp.print(j + 1);
      Logp.print("_tx,Port");
      Logp.print(j + 1);
      Logp.print("_rx,Port");
      Logp.print(j + 1);
      Logp.print("_dlt,Port");
      Logp.print(j + 1);
      Logp.print("_incorr");
      if ((j + 1) != N_PORTS) {
        Logp.print(",");
      }
    }
  }

  Logp.println();
  if ( diag_poll_cntr < 10) {
    Logp.print("0");
  }
  Logp.print(diag_poll_cntr);
  Logp.print(',');
  Logp.print(st_mkr_name);
  Logp.print(",  ");

  for (int i = 0; i < N_PORTS; i++) {
    Logp.print(rs_ports[i].port_settings, HEX);
    Logp.print("h,");
    Logp.print(rs_ports[i].n_tx);
    Logp.print(",");
    Logp.print(rs_ports[i].n_rx);
    Logp.print(", e");
    Logp.print(rs_ports[i].n_tx - rs_ports[i].n_rx);
    Logp.print(", ic");
    Logp.print(rs_ports[i].n_incorr);
    if ((i + 1) != N_PORTS) {
      Logp.print(", ");
    }
  }
  Logp.println();
  ++diag_poll_cntr;
  //digitalWrite(LED_BUILTIN, LOW);

}
