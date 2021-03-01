interface DynamicSchedule{

  async command void Initial(uint8_t Task_charater[][4], uint8_t schedule_len);

  async command uint32_t Scheduler(uint8_t schedule[][11], uint8_t row , uint8_t flowid_root ,uint8_t flowid, uint8_t hopcount, uint8_t schedule_len);


}
