module IntegrateScheduleP {
  provides{
    interface IntegrateSchedule;
  }

  uses{
    interface Useful;
    interface DynamicSchedule;
  }

}
implementation {

  uint8_t Find_index(uint8_t schedule[][11], uint8_t flowid, uint8_t hopcount, uint8_t schedule_len);

  //EDF Schedule
  //***flowid_root["2"] 2==TaskNumber
  async command void IntegrateSchedule.SetSchedule(uint8_t schedule[][11],uint8_t Task_charater[][4],uint8_t schedule_len,  uint8_t flowid_root[2] ,uint8_t TaskNumber, uint8_t hopcount_len[3])
  { //***hopcount_len["3"] == TaskNumber + 1
    uint8_t i,j;
    uint32_t tmpSlot;
    uint8_t tmpSenderReceiver[2];
    uint8_t index;
    uint8_t rowcount;

    rowcount = 0;

    call DynamicSchedule.Initial(Task_charater, schedule_len);    //Initialize Schedule
    for(i=1; i<TaskNumber+1; i++)
    {
      for(j=1; j<hopcount_len[i]+1; j++)
      {
        tmpSlot = call DynamicSchedule.Scheduler(schedule, rowcount, flowid_root[i-1], i,j, schedule_len);  //i==flowid , j==hopcount   EDF Schedule
        call Useful.SetSR(tmpSenderReceiver,1, i,j);    // 1==data transmission, 0==control trnasmission
                                                        // set sender,receiver according to cuurent flowid and hopcount

        index = Find_index(schedule, i, j, schedule_len);   //find index according current Taskid(flowid) and hopcount
        schedule[index][0] = tmpSlot;
        schedule[index][1] = tmpSenderReceiver[0];    //sender setting
        schedule[index][2] = tmpSenderReceiver[1];    //receiver setting
                                                      //hopcount setting
        //printf("j:%u\r\n",j);                         //^^
        //printfflush();
        /*
        if(i == 2 && j==1)
        {
          printf("sender:%u, receiver:%u\r\n",tmpSenderReceiver[0],tmpSenderReceiver[1]);
          printfflush();
        }
        */

        //printf("**id:%u type: data_trans, taskid: %u, hopcount: %u, slot:%u, sender:%u, receiver:%u \n",TOS_NODE_ID,i,j,tmpSlot,tmpSenderReceiver[0],tmpSenderReceiver[1]);
        //printf("**id:%u type: data_trans, taskid: %u, hopcount: %u, slot:%u, index:%u sender:%u, receiver:%u \n",TOS_NODE_ID,i,j,tmpSlot,index, schedule[index][1],schedule[index][2]);

        rowcount = rowcount + 1;
      }

    }
    rowcount = 0;

    /*
    printf("schedule:\r\n");
    printfflush();
    for(i=0; i<schedule_len; i++)
    {
      printf("[%u]: {%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u}\r\n",i, schedule[i][0],schedule[i][1],schedule[i][2],schedule[i][3],schedule[i][4],schedule[i][5],schedule[i][6],schedule[i][7],schedule[i][8],schedule[i][9],schedule[i][10]);
      printfflush();
    }
    */


  }

  uint8_t Find_index(uint8_t schedule[][11], uint8_t flowid, uint8_t hopcount, uint8_t schedule_len)  //get index to change original schedule information
  {
    uint8_t i,j;
    uint8_t index=0;

    for(i=0; i<schedule_len; i++)
    {
      if((schedule[i][6] == flowid) && (schedule[i][10] == hopcount))
      {
        index = i;
        return index;
      }
    }
    //printf("error Find_index!\n");
    return schedule_len;
  }


}
