#ifndef BKKPROTO_H
#define BKKPROTO_H

#define PFX             {'s','L'}
#define PFX_OFFSET      0
#define PFX_SIZE        2

#define TYPE_OFFSET     2
#define TYPE_SIZE       1
#define CNTR_OFFSET     (PFX_SIZE+TYPE_SIZE)
#define CNTR_SIZE       6
#define RESERVE_OFFSET  (CNTR_OFFSET+CNTR_SIZE)
#define PART_CNTR_OFFSET (RESERVE_OFFSET+1)
#define PART_SIZE_OFFSET (PART_CNTR_OFFSET+1)

#define HEADER_SIZE     10
#define PHD_SIZE        (PFX_SIZE+HEADER_SIZE)

#define PACK_TYPE_NO        0
#define PACK_TYPE_SHA1      1
#define SHA1_HMAC_SIZE      20
#define PACK_TYPE_SHA256    2
#define SHA256_HMAC_SIZE    32
#define PACK_TYPE_CHACHA20  3
#define CHACHA20_HMAC_SIZE  16
#define CRC16_SIZE          2

#define PACK_TYPE_FLAG_REQUEST      0x10
#define PACK_TYPE_FLAG_ACKNOWLEDGE  0x20

#define SPD9600     0x10
#define SPD19200    0x20
#define SPD38400    0x30
#define SPD57600    0x40
#define SPD115200   0x50
#define SPD230400   0x60
#define SPD460800   0x70

#define RS_SERIAL_8N1      0x01
#define RS_SERIAL_8N2      0x02
// 0x04|0x01 = 0x05
#define RS_SERIAL_8O1      0x05
// 0x04|0x02 = 0x06
#define RS_SERIAL_8O2      0x06
// 0x08|0x01 = 0x09
#define RS_SERIAL_8E1      0x09
// 0x08|0x02 = 0x0a
#define RS_SERIAL_8E2      0x0a

#define N_DEVS_MAX  30
#define MAX_DEV_TX  256
#define MAX_DEV_RX  256
#define MAX_PACK_SIZE  (PFX_SIZE+HEADER_SIZE+SHA256_HMAC_SIZE+400)

#define PAYLOAD_TYPE_NONE       0x0
#define PAYLOAD_TYPE_COMMAND    0x0c
#define PAYLOAD_TYPE_GET_BUFFER 0x0b
#define PAYLOAD_TYPE_ANSWER     0x0a

#define CLT_MAX_AGE_TICS  8


int hmac_size(int typ);
#endif //BKKPROTO_H
