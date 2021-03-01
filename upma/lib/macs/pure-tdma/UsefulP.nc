module UsefulP {
  provides{
    interface Useful;
  }
}
implementation {

//broadcast : for the control message
//uplink : data to gw
//downlink : gw to data

  //uint8_t CharacterNumber = 3;
  uint8_t TaskNumber = 2;
  uint8_t DEADLINE = 2;

  //used by TaskSenderReceiver function.
  uint8_t FLOWID = 0;
  uint8_t SENDER = 1;
  uint8_t RECEIVER = 2;
  uint8_t HOPCOUNT = 3;

  uint8_t NetworkRoute[7][7]={      //***NodeNumber $$when this function work?
  {0,0,1,0,1,0,1},
  {0,0,1,1,1,0,0},
  {1,1,0,1,1,0,0},
  {0,1,1,0,1,1,1},
  {1,1,1,1,0,1,1},
  {0,0,0,1,1,0,1},
  {1,0,0,1,1,1,0}
  };

  //Topology_information
  uint8_t makeroute[7][7]={         //*** 7==NodeNumber
  {0,0,0,0,1,0,1},    //0
  {0,0,1,0,0,0,0},
  {1,0,0,0,0,0,0},
  {0,0,0,0,0,0,0},
  {0,0,0,1,0,0,0},     //4
  {1,0,0,0,0,0,0},
  {0,0,0,0,0,0,0}
  //{0,0,0,0,0,1,0,0},
  //{0,0,0,0,0,0,1,0},
  //{1,0,0,0,0,0,0,0}
  //{0,0,0,0,0,0,0,0,0,1,0},
  //{0,0,0,0,0,0,0,0,0,0,1},
  //{1,0,0,0,0,0,0,0,0,0,0},
  //{1,0,0,0,0,0,0,0,0,0,0}
  };

  

    //This func is not used now
    async command uint8_t* Useful.DeadlineList(uint8_t Pkt_Character[3])
    {
      uint8_t i;
      uint8_t rtn[TaskNumber];

      for(i;i<TaskNumber;i++)
      {
        rtn[i] = Pkt_Character[DEADLINE];
      }

      printf("DeadlineList is operated successfully!\n");

      return rtn;

    }//return integrated deadline of all tasks

    //This func is not used now
    async command uint8_t* Useful.TaskSenderReceiver(uint8_t flowid, uint8_t hopcount)
    {
      uint8_t sender;
      uint8_t receiver;
      uint8_t rtn[2];


      if(flowid == 1 && hopcount == 1)
      {
        sender = 11;
        receiver = 12;
      }else if(flowid == 1 && hopcount == 2)
      {
        sender = 12;
        receiver = 10;
      }else if(flowid == 2 && hopcount == 1)
      {
        sender = 13;
        receiver = 14;
      }else if(flowid == 2 && hopcount == 2)
      {
        sender = 14;
        receiver = 10;
      }else if(flowid == 3 && hopcount == 1)
      {
        sender = 15;
        receiver = 16;
      }else if(flowid == 3 && hopcount == 2)
      {
        sender = 16;
        receiver = 10;
      }else
      {
        sender = 100;
        receiver = 100;
      }

      rtn[0]=sender;
      rtn[1]=receiver;


      //printf("TaskSenderReceiver=> rtn[0]: %u, rtn[1]: %u, rtn[2]: %u, rtn[3]: %u\n",rtn[0],rtn[1],rtn[2],rtn[3]);
      return rtn;
      //retrun [0]:sender  [1]:receiver
    }

    //This func is not used now
    async command void Useful.MakeRoute(uint8_t transmission_type, uint8_t graph[][5])  //***5 == NodeNumber
    {
      uint8_t NodeNumber = 5;     //***
      uint8_t IDoffset = 10;
      uint8_t i,j;
      uint8_t rtn[5][5];    //***

      if(transmission_type == 1)  //data transmission
      {
        for(i=0; i<NodeNumber; i++)
        {
          for(j=0; j<NodeNumber; j++)
          {
            graph[i][j] = makeroute[i][j];
          }
        }
      }else   //control transmission
      {
      for(i=0; i<NodeNumber; i++)
      {
        for(j=0; j<NodeNumber; j++)
        {
          graph[i][j] = makeroute[j][i];
        }
      }
      }


      //return rtn;
    }

    //set sender,receiver according to input(flowid, hopcount)
    async command void Useful.SetSR(uint8_t tmpSenderReceiver[2], uint8_t transmission_type, uint8_t flowid, uint8_t hopcount)
    {
      uint8_t Nodeoffset = 0;  //&&
      uint8_t NodeNumber = 7;     //***
      uint8_t sender, receiver;
      uint8_t i,j;
      uint8_t tmp;
      uint8_t max_flow_len;
      uint8_t flow[5] = {0,0,0,0,0};  //***5 == max_flow_len      ex. 1->2->0->4->3   {1,2,0,4,3}
      uint8_t data_totalflow[2][5] = {  //***first "2" == Flow(Task) Nummber, second "5" == max_flow_len
      {1,2,0,4,3},    //flow 1 routing path
      {5,0,6}    //flow 2 routing path
      //{5,6,8,10,0}
      };
      uint8_t control_totalflow[1][3] = {   //***
      {0,2,4}
      //{0,4,3}
      //{0,10,8,6,5}
      };

      max_flow_len = 5;     //***
      sender = 0;
      receiver = 0;
      tmp = 0;



      if(transmission_type == 1)    //1==data transmission
      {
        if(flowid == 1)               //***When we change # of flow, we need to revise like below form
        {
          for(i=0;i<max_flow_len;i++)
          {
            flow[i] = data_totalflow[flowid-1][i];
          }
          if((hopcount == 1) || (hopcount == 2) || (hopcount == 3)|| (hopcount == 4))        //*** Need to revise when you change network topology, ex. hopcount = 4 , in case of 1->2->0->4->3
          {
            if(makeroute[flow[hopcount-1]][flow[hopcount]] == 1)
            {
              sender = flow[hopcount-1];
              receiver = flow[hopcount];
            }else
            {
              //printf("error in makeroute!\n");
              sender = 0;
              receiver = 0;
            }

          }else
          {
            //printf("Check hopcount in SetSR!\n");
            sender = 0;
            receiver = 0;
          }

        }
        else if(flowid == 2)       //***
        {
          for(i=0;i<max_flow_len;i++)
          {
            flow[i] = data_totalflow[flowid-1][i];
          }
          if((hopcount == 1) || (hopcount == 2))    //*** revise hopcount according to # of hopcount  || (hopcount == 2)|| (hopcount == 3)|| (hopcount == 4)
          {
            if(makeroute[flow[hopcount-1]][flow[hopcount]] == 1)
            {
              sender = flow[hopcount-1];
              receiver = flow[hopcount];
            }else
            {
              printf("error in makeroute!\n");
              sender = 0;
              receiver = 0;
            }
          }else
          {
            printf("Check hopcount in SetSR!\n");
            sender = 0;
            receiver = 0;
          }
        }

        /*
        else if(flowid == 3)   //***    if you use flow 3, use this part
        {
          for(i=0;i<max_flow_len;i++)
          {
            flow[i] = data_totalflow[flowid-1][i];
          }
          if((hopcount == 1) || (hopcount == 2) || (hopcount == 3)|| (hopcount == 4))   //***
          {
            if(makeroute[flow[hopcount-1]][flow[hopcount]] == 1)
            {
              sender = flow[hopcount-1];
              receiver = flow[hopcount];
            }else
            {
              printf("error in makeroute!\n");
              sender = 0;
              receiver = 0;
            }
          }else
          {
            printf("Check hopcount in SetSR!\n");
            sender = 0;
            receiver = 0;
          }
        }*/
        else
        {
          sender = 0;
          receiver = 0;
        }

      }//end data transmission


      tmpSenderReceiver[0] = sender + Nodeoffset;
      tmpSenderReceiver[1] = receiver + Nodeoffset;

      //printf("**flowid: %u, hopcount: %u ; sender: %u, receiver: %u\n",flowid, hopcount, tmpSenderReceiver[0], tmpSenderReceiver[1]);
    }

}
