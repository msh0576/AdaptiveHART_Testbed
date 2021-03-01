interface IntegrateSchedule{

  async command void SetSchedule(uint8_t schedule[][11],uint8_t Task_charater[][4],uint8_t schedule_len, uint8_t flowid_root[2] , uint8_t TaskNumber, uint8_t hopcount_len[3]);
  //***flowid_root["2"] 2==TaskNumber ,   hopcount_len["3"] == TaskNumber + 1
}
