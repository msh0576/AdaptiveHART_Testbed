configuration SchedulerC {
  provides{
    interface DynamicSchedule;
  }

  
}
implementation {

  components SchedulerP;
  components UsefulC;

  DynamicSchedule = SchedulerP.DynamicSchedule;
  SchedulerP.Useful -> UsefulC.Useful;
}
