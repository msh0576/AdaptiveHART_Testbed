
module RoutingTableP{

  provides{
    interface Routing;
  }

}
implementation{
am_addr_t *neighbor;

  //  level 1 :       5          6
  //  level 0 :            0
  //  level 1 :       2         4
  //  level 2 :     1               3
  async command am_addr_t Routing.MakeParent(am_addr_t nodeId, am_addr_t *Neighbor)   //***
  {   //This function is used to transmit control msg to every node.
      //In our setting, node 2 , 4 are relay nodes, we need to establish their parent node 1 , 3
    am_addr_t parentId;

    if(nodeId==4)
    {
      parentId=3;
    }
    else if(nodeId==3)
    {
      parentId=100;
    }
    else if(nodeId==2)
    {
      parentId=1;
    }
    else if(nodeId==1)
    {
      parentId=100;
    }
    else
    {
      parentId=0;
    }


    return parentId;
  }

  async command am_addr_t * Routing.NeighborTable(am_addr_t nodeId)
  {


    //neighbor[0]=11;
    //neighbor[1]=12;
    if(nodeId==4)
    {
      neighbor[0]=2;
      neighbor[1]=3;
    }
    else if(nodeId==3)
    {
      neighbor[0]=1;
      neighbor[1]=2;
      neighbor[2]=4;
    }
    else if(nodeId==2)
    {
      neighbor[0]=1;
      neighbor[1]=3;
      neighbor[2]=4;
    }
    else if(nodeId==1)
    {
      neighbor[0]=0;
      neighbor[1]=2;
      neighbor[2]=3;
    }
    else
    {
      neighbor[0]=1;
    }

    return (am_addr_t*)neighbor;
  }

}
