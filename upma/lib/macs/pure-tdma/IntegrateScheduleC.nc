configuration IntegrateScheduleC {
  provides{
    interface IntegrateSchedule;
  }


}
implementation {
  components IntegrateScheduleP;
  components SchedulerC;
  components UsefulC;

  IntegrateSchedule = IntegrateScheduleP.IntegrateSchedule;
  IntegrateScheduleP.Useful -> UsefulC.Useful;
  IntegrateScheduleP.DynamicSchedule -> SchedulerC.DynamicSchedule;
}
