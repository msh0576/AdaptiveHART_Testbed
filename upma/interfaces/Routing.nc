
interface Routing
{

  async command am_addr_t MakeParent(am_addr_t nodeId, am_addr_t *Neighbor);

  async command am_addr_t * NeighborTable(am_addr_t nodeId);

}
