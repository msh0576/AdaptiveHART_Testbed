configuration SchedulerC {
  provides{
    interface Useful;
  }
}
implementation {

  components UsefulP;

  Useful = UsefulP.Useful;

}
