

module ScheduleSlotP{
  provides{
    interface MySlot;
  }

}
implementation{


  async command uint8_t MySlot.getmyslot(am_addr_t * nodeid){
    uint8_t slot;

      slot=TOS_NODE_ID;
 
    return slot;
  }


}
