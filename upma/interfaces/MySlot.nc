#include <message.h>

interface MySlot {

  async command uint8_t getmyslot(am_addr_t * nodeid);

}
