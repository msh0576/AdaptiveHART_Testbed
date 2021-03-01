
module SchedulerP {
  provides{
    interface DynamicSchedule;
  }

  uses{
    interface Useful;
  }

}
implementation {

/****************************************************************
*      want to identify whether each task is allocated in a slot,
*      debug at &&*
*
*/
//*******************************************************************

  uint8_t TaskNumber = 2;   //***
  uint8_t CharacterNumber = 4;  //doesn't need to revise "4"=Task_charater_len

  uint8_t TASK = 0;
  uint8_t PERIOD = 1;
  uint8_t DEADLINE = 2;
  uint8_t REMAINHOP = 3;

  uint8_t Channel_num = 1;




  void LeastDeadline(uint8_t* Pkt_deadline_list, uint8_t* Remainhop_list);
  void DeadlineList(uint8_t Pkt_deadline[][CharacterNumber]);
  void TaskList(uint8_t Pkt_Task[][CharacterNumber]);
  void RemainhopList(uint8_t Pkt_Task[][CharacterNumber]);
  void ScheduleEDF(uint8_t schedule_len);
  uint32_t FindTask(uint8_t flowid, uint8_t hopcount, uint8_t schedule_len);

  //***  2 == TaskNumber
  uint8_t tmpDeadlineList[2]; //***
  uint8_t TotalTaskList[2]; //***
  uint8_t tmpRemainhopList[2];  //***
  uint8_t LeastDeadline_Task[1];
  uint8_t schedule[6];   //*** 6 == schedule_len
  //*** 2 == TaskNumber
  uint8_t BackDeadlineList[2];  //***
  uint8_t BackTotalTaskList[2]; //***
  uint8_t BackRemainhopList[2]; //***

  async command void DynamicSchedule.Initial(uint8_t Task_charater[][4], uint8_t schedule_len)    //[0]: Taskid,  [1]: Pkt period,  [2]: Pkt Deadline,  [3]: remain hop
  {
    uint8_t i;

    //Make List suchas dealine, taskid, remainhop...
    DeadlineList(Task_charater);
    TaskList(Task_charater);
    RemainhopList(Task_charater);
    //
    for(i=0; i<TaskNumber; i++)
    {
      BackDeadlineList[i] = tmpDeadlineList[i];
      BackTotalTaskList[i] = TotalTaskList[i];
      BackRemainhopList[i] = tmpRemainhopList[i];
    }
    //
    LeastDeadline(tmpDeadlineList, tmpRemainhopList);   //find min dealine task among all tasks
    ScheduleEDF(schedule_len);    //allocate taskid in a slot, which is from slot 1 to slot schedule_len(superframe length).

  }

  async command uint32_t DynamicSchedule.Scheduler(uint8_t schedule[][11], uint8_t row , uint8_t flowid_root ,uint8_t flowid, uint8_t hopcount, uint8_t schedule_len)
  {   //allocate schedule info in a schedule[][] from "PureTDMASchedulerP".
    //uint8_t* LeastDeadline_Task;
    uint8_t i,j;
    uint32_t slot;
    uint8_t* tmpSenderReceiver;

    slot = 0;
    i=0;

    slot = FindTask(flowid, hopcount, schedule_len);

    schedule[row][6] = flowid;
    schedule[row][10] = hopcount;

    //&&& default setting
    schedule[row][3] = 22; //channels
    schedule[row][4] = 0;   //Accesstype : TDMA
    schedule[row][5] = 1;   //flowtype: regular
    schedule[row][7] = flowid_root;    //flow root
    schedule[row][8] = 0;    //send status
    schedule[row][9] = 0;   //last hop status


    for(i=0; i<TaskNumber; i++)
    {
      tmpDeadlineList[i] = BackDeadlineList[i];
      TotalTaskList[i] = BackTotalTaskList[i];
      tmpRemainhopList[i] = BackRemainhopList[i];
    }

    //printf("End DynamicSchedule\n");
    return slot;
  }





  //Deadline - Remainhop
  void LeastDeadline(uint8_t* Pkt_deadline_list, uint8_t* Remainhop_list)
  {
    uint8_t current_min_deadline_index = 0;
    uint8_t i = 0;
    uint8_t tmpPkt_deadline;
    uint8_t tmp[2];  //*** 2=TaskNumber

    for(i=0; i<TaskNumber; i++)
    {
      tmp[i] = Pkt_deadline_list[i] - Remainhop_list[i];
    }
    //printf("deadline - remainhop = %u, %u, %u\n",tmp[0],tmp[1],tmp[2]);

    tmpPkt_deadline = 100;
    //printf("tmpPkt_deadline: %u\n",tmpPkt_deadline);
    for(i=0; i<TaskNumber; i++)
    {
      if(tmpPkt_deadline - tmp[i] >= 0)
      {
        tmpPkt_deadline = tmp[i];
        current_min_deadline_index = i;
      }
    }

    LeastDeadline_Task[0] = tmpPkt_deadline;
    LeastDeadline_Task[1] = current_min_deadline_index;

  }// return [0]:deadline, [1]:index

  void RemainhopList(uint8_t Pkt_Task[][CharacterNumber])
  {
    uint8_t i=0;

    for(i=0;i<TaskNumber;i++)
    {
      tmpRemainhopList[i] = Pkt_Task[i][REMAINHOP];
    }

  }

  void DeadlineList(uint8_t Pkt_deadline[][CharacterNumber])
  {
    uint8_t i=0;

    for(i;i<TaskNumber;i++)
    {

      tmpDeadlineList[i] = Pkt_deadline[i][DEADLINE];
    }



  }//return integrated deadline of all tasks




  void TaskList(uint8_t Pkt_Task[][CharacterNumber])
  {
    uint8_t i=0;

    //printf("TaskList:");
    for(i;i<TaskNumber;i++)
    {
      TotalTaskList[i] = Pkt_Task[i][TASK];
      //printf("%u,  ", TotalTaskList[i]);
    }
    //printf("\n");
    //printf("All Task List=> %u, %u, %u\n", TotalTaskList[0], TotalTaskList[1], TotalTaskList[2]);

  }//return integrated task id of all tasks





  void ScheduleEDF(uint8_t schedule_len)
  {
    uint8_t slot = 0;
    uint8_t i = 0;

    //printf("EDF schedule[slot]: ");
    for(slot = 1; slot < schedule_len+1; slot++)
    {

      if(TotalTaskList[LeastDeadline_Task[1]] != 100)       //check available task
      {
        schedule[slot] = TotalTaskList[LeastDeadline_Task[1]];                    //least EDF task allocation at Schedule[slot]
        tmpRemainhopList[LeastDeadline_Task[1]] = tmpRemainhopList[LeastDeadline_Task[1]] - 1;    //decrease remain hop

        //printf("id: %u  LeastDeadline_Task : [0]%u   [1]:%u \n", TOS_NODE_ID, LeastDeadline_Task[0], LeastDeadline_Task[1]);
        //&&*
        //printf("id: %u   schedule Task: %u at %u  remainhop to destination: %u\r\n",TOS_NODE_ID,schedule[slot], slot, tmpRemainhopList[LeastDeadline_Task[1]]);
        //&&* //^^

        if(tmpRemainhopList[LeastDeadline_Task[1]] == 0)            //check last hop
        {

          TotalTaskList[LeastDeadline_Task[1]] = 100;                              //remove task  in TotalTaskList
          tmpDeadlineList[LeastDeadline_Task[1]] = 100;                             //remove deadline


          LeastDeadline(tmpDeadlineList, tmpRemainhopList);      //recalculate leastdeadline without TotalTaskList == 100 !
          //printf("id: %u  LeastDeadline_Task : [0]%u   [1]:%u \n", TOS_NODE_ID, LeastDeadline_Task[0], LeastDeadline_Task[1]);

        }

      }else
      {
        //printf("dummy at slot:%u!\n",slot);
        schedule[slot] = 100;
      }

      /*
      if(LeastDeadline_Task[0] == 100)
      {
        //printf("break at slot:%u!\n",slot);
        break;
      }
      */
        //printf("schedule[%u]:%u, \r\n",slot,schedule[slot]);//^^
    }
    //printf("\n");
    //printf("After EDF, Deadline List: [0]:%u, [1]:%u, [2]:%u \n",tmpDeadlineList[0],tmpDeadlineList[1],tmpDeadlineList[2]);
    return schedule;
  }


  uint32_t FindTask(uint8_t flowid, uint8_t hopcount, uint8_t schedule_len)
  {
    uint8_t i;
    uint32_t slot = 0;
    uint8_t tmphopcount = 0;

    tmphopcount = hopcount;


    for(i=1; i<schedule_len+1; i++)
    {
      if((tmphopcount != 1) && (schedule[i] == flowid))
      {
        tmphopcount = tmphopcount - 1;
      }else if((schedule[i] == flowid) && (tmphopcount == 1))   //find slot according to flow id, hopcount
      {
        slot = i;
        //printf("&current slot: %u\n",slot);
        break;
      }else   //can't find slot
      {
        slot = 100;
      }


    }

    //printf("slot: %u  , flowid: %u  , hopcount:%u\r\n",slot,flowid,hopcount);
    //printfflush();
    //return slot according to flowid and hopcount
    return slot;
  }


}
