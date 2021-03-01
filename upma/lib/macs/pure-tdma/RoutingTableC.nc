
configuration RoutingTableC{
  provides{
    interface Routing;
  }

}
implementation{
  components RoutingTableP;

  Routing=RoutingTableP.Routing;

}
