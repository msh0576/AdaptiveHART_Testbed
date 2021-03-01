
configuration ScheduleSlotC{
  provides{
    interface MySlot;
  }
}
implementation{
  components ScheduleSlotP;

  MySlot = ScheduleSlotP.MySlot;

}
