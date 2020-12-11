#include "bkkproto.h"


int hmac_size(int typ){
  int retval = 0;
  switch(typ){
    case 0:
      retval = 0;
      break;
    case PACK_TYPE_SHA1:
      retval = SHA1_HMAC_SIZE;
      break;
    case PACK_TYPE_SHA256:
      retval = SHA256_HMAC_SIZE;
      break;
    case PACK_TYPE_CHACHA20:
      retval = CHACHA20_HMAC_SIZE;
      break;
  }  
  return retval;
}
