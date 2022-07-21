# A class to store job details. Each job has an identifier,
# a deadline, and profit associated with it.
from datetime import datetime, timedelta, date
class Job:
    def __init__(self, taskId, deadline, profit, burstTime):
        isinstance(burstTime, timedelta)
        isinstance(deadline, datetime)
        isinstance(profit, int)
        self.taskId = taskId
        self.deadline = deadline
        self.profit = profit
        self.burstTime = burstTime
 
class TimeSlot:
    def __init__(self, start=datetime.combine(date.today(), datetime.min.time()), 
                        stop=datetime.combine(date.today(), datetime.max.time()), 
                        state=False, superSlot=None):
        isinstance(start, datetime)
        isinstance(stop, datetime)
        # True is used, False is free
        isinstance(state, bool)
        self.start = start
        self.stop = stop
        # True is used, False is free
        self.state = state
        self.subSlots = []
        self.superSlot = superSlot
        self.print_indent = 0
    
    def bookSlot(self, deadline_timeStamp, burst_time):
        isinstance(deadline_timeStamp, datetime)
        isinstance(burst_time, timedelta)
        booked_slot = None
        if self.within(deadline_timeStamp):
            self.state = True
            self.subSlots.append(TimeSlot(self.start, deadline_timeStamp-burst_time, state=False, superSlot=self))
            booked_slot = TimeSlot(deadline_timeStamp-burst_time, deadline_timeStamp, state=True, superSlot=self)
            self.subSlots.append(booked_slot)
            self.subSlots.append(TimeSlot(deadline_timeStamp,self.stop, state=False, superSlot=self))
        else:
            raise Exception("Requested division of TimeSlot in TimeStamp out of the TimeSlot")
        return booked_slot
    
    def within(self, timeStamp):
        isinstance(timeStamp, datetime)
        return (timeStamp >= self.start) and (timeStamp <= self.stop)

    def get_slot_by_datetime(self, dt):
        isinstance(dt, datetime)
        if self.state == False and self.within(dt):
            return self
        else:
            for subslot in self.subSlots:
                if subslot.within(dt) and subslot.state == False:
                    ss = subslot.get_slot_by_datetime(dt)
                    # if ss != None:
                    return ss
        return None

    def print_slots(self, print_indent=0):
        print (str(self.start)+">>"+str(self.stop))
        s = ''
        for slot in self.subSlots:
            slot.print_slots(print_indent+1)
            for i in range(print_indent):
                s += '  '
            print (s+str(slot.start)+">>"+str(slot.stop))
# class TimeSlots:
#     def __init__(self):
#         self.slots = []

#     def add_slot(self, slot):
#         isinstance(slot, TimeSlot)
#         self.slots.append(slot)

#     def get_slot_by_datetime(self, dt):
#         isinstance(dt, datetime)
#         for slot in self.slots:
#             if slot.within(dt): 
#                 return slot
#         return None

# Function to schedule jobs to maximize profit
def scheduleJobs(jobs, T):
 
    # stores the maximum profit that can be earned by scheduling jobs
    profit = 0
 
    # list to store used and unused slots info
    day_slot = TimeSlot(datetime.combine(date.today(), datetime.min.time()), datetime.combine(date.today(), datetime.max.time()))
    day_slot.print_slots()
    # arrange the jobs in decreasing order of their profits
    jobs.sort(key=lambda x: x.profit, reverse=True)
    j_slot = TimeSlot(state=True)
    # consider each job in decreasing order of their profits
    for job in jobs:
        # search for the next free slot and map the task to that slot
        # for s in reversed(range(job.deadline)):
        print ("JOB: "+str(job.taskId))
        day_slot.print_slots()
        j_slot = day_slot.get_slot_by_datetime(job.deadline)
        if j_slot == None:
            print("Cannot book slot")
        else:
            my_slot = j_slot.bookSlot(job.deadline, job.burstTime)
            print (str(my_slot.start)+">>"+str(my_slot.stop))
        # while j_slot.state:
        #     j_slot = day_slot.get_slot_by_datetime(job.deadline)

        # if j_slot.state:
        #     if j < T and slot[j] == -1:
        #         slot[j] = job.taskId
        #         profit += job.profit
        #         break
 
    # print the scheduled jobs
    print('The scheduled jobs are', list(filter(lambda x: x != -1, slot)))
 
    # print total profit that can be earned
    print('The total profit earned is', profit)
 
 
if __name__ == '__main__':
 
    # List of given jobs. Each job has an identifier, a deadline, and
    # profit associated with it
    jobs = [
        Job(1, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=9), 15, timedelta(minutes=2)), 
        Job(2, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2), 2, timedelta(minutes=2)), 
        Job(3, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5), 18, timedelta(minutes=2)), 
        Job(4, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=7), 1, timedelta(minutes=2)), 
        Job(5, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4), 25, timedelta(minutes=2)),
        Job(6, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2), 20, timedelta(minutes=2)), 
        Job(7, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5), 8, timedelta(minutes=2)), 
        Job(8, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=7), 10, timedelta(minutes=2)), 
        Job(9, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4), 12, timedelta(minutes=2)), 
        Job(10, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=3), 5, timedelta(minutes=2))
    ]
 
    # stores the maximum deadline that can be associated with a job
    T = 15
 
    # schedule jobs and calculate the maximum profit
    scheduleJobs(jobs, T)