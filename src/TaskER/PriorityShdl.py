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
                        state=False, superSlot=None, job_id=None):
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
        self.minimalSlot = timedelta(seconds=5)
        self.jobID = job_id

    def getDuration(self):
        return self.stop - self.start
    
    def bookSlot(self, deadline_timeStamp, burst_time, job_id):
        isinstance(deadline_timeStamp, datetime)
        isinstance(burst_time, timedelta)
        booked_slot = None
        if self.within(deadline_timeStamp, burst_time) and self.state==False:
            self.state = True
            if self.getDuration() - burst_time <= self.minimalSlot:
                self.jobID = job_id
                return self
            pre_slot = TimeSlot(self.start, deadline_timeStamp-burst_time, state=False, superSlot=self)
            print("pre: "+str(pre_slot.getDuration()))
            if pre_slot.getDuration() > self.minimalSlot:
                self.subSlots.append(pre_slot)
            booked_slot = TimeSlot(deadline_timeStamp-burst_time, deadline_timeStamp, state=True, superSlot=self, job_id=job_id)
            self.subSlots.append(booked_slot)
            post_slot = TimeSlot(deadline_timeStamp,self.stop, state=False, superSlot=self)
            if post_slot.getDuration() > self.minimalSlot:
                self.subSlots.append(post_slot)
        else:
            raise Exception("Requested division of TimeSlot in TimeStamp out of the TimeSlot")
        return booked_slot
    
    def within(self, timeStamp, burstTime):
        isinstance(timeStamp, datetime)
        isinstance(burstTime, timedelta)
        # print("ts: ", timeStamp, "bt: ", burstTime, "d: ", self.getDuration())
        if (timeStamp - burstTime >= self.start) and (timeStamp <= self.stop) \
                    and (self.getDuration()>=burstTime):
            # print("within")
            return True
        else:
            return False

    def get_slot_by_datetime(self, dt, burst_time):
        isinstance(dt, datetime)
        isinstance(burst_time, timedelta)
        if self.state == False and self.within(dt, burst_time):
            # print("self")
            return self
        else:
            for subslot in self.subSlots:
                # print("subslot:", subslot.start)
                if subslot.within(dt, burst_time) and subslot.state == False:
                    # print("subslot: ", subslot.start)
                    return subslot
                else:
                    # print("else")
                    ss = subslot.get_slot_by_datetime(dt, burst_time)
                    if ss != None:
                        return ss# if ss != None:
                    continue
        return None

    def print_slots(self, print_indent=1):
        # s = ''
        # for i in range(print_indent):
        #     s += '  '
        #     i+=1
        # print (s+str(self.start)+">>"+str(self.stop))
        for slot in self.subSlots:
            s = ''
            for i in range(print_indent):
                s += '  '
                i+=1
            print (s+str(slot.start)+">>"+str(slot.stop)+" state: "+str(slot.state))
            slot.print_slots(print_indent+1)
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
class PriorityScheduler():
    def __init__(self):
        self.jobs = []

    def addJob(self, job):
        self.jobs.append(job)

    class Schedule():
        def __init__(self):
            self.scheduled = []
            self.rejected = []
    def scheduleJobs(self):
    
        # stores the maximum profit that can be earned by scheduling jobs
        profit = 0
    
        # list to store used and unused slots info
        day_slot = TimeSlot(datetime.combine(date.today(), datetime.min.time()), datetime.combine(date.today(), datetime.max.time()))
        day_slot.print_slots()
        # arrange the jobs in decreasing order of their profits
        self.jobs.sort(key=lambda x: x.profit, reverse=True)
        j_slot = TimeSlot(state=True)
        # consider each job in decreasing order of their profits
        slots = []
        profit = 0
        shdl_result = self.Schedule()
        for job in self.jobs:
            # search for the next free slot and map the task to that slot
            # for s in reversed(range(job.deadline)):
            print ("JOB: "+str(job.taskId)+" ----"+str(job.deadline))
            j_slot = day_slot.get_slot_by_datetime(job.deadline, job.burstTime)
            # print("j_slot ", j_slot)
            if j_slot == None:
                shdl_result.rejected.append(job.taskId)
                print("Cannot book slot")
            else:
                my_slot = j_slot.bookSlot(job.deadline, job.burstTime, job.taskId)
                shdl_result.scheduled.append(my_slot)
                profit += job.profit
                # print (str(my_slot.start)+">>"+str(my_slot.stop))
            day_slot.print_slots()
            print()
            # while j_slot.state:
            #     j_slot = day_slot.get_slot_by_datetime(job.deadline)

            # if j_slot.state:
            #     if j < T and slot[j] == -1:
            #         slot[j] = job.taskId
            #         profit += job.profit
            #         break
    
        # print the scheduled jobs
        print('The scheduled jobs are:')
        for i in shdl_result.scheduled:
            print(str(i.start)+">>"+str(i.stop)+" ---->"+str(i.jobID))
        print('The rejected jobs are:')
        for i in shdl_result.rejected:
            print(str(i))
    
        # print total profit that can be earned
        print('The total profit earned is', profit)
        return shdl_result
    
 
# if __name__ == '__main__':
 
#     # List of given jobs. Each job has an identifier, a deadline, and
#     # profit associated with it
#     jobs = [
#         Job(1, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=9), 15, timedelta(minutes=3)), 
#         Job(2, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2), 2, timedelta(minutes=1)), 
#         Job(3, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5), 18, timedelta(minutes=2)), 
#         Job(4, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=10), 1, timedelta(minutes=3)), 
#         Job(5, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4), 25, timedelta(minutes=5)),
#         Job(6, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2), 20, timedelta(minutes=1)), 
#         Job(7, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5), 8, timedelta(minutes=1)), 
#         Job(8, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=7), 10, timedelta(minutes=1)), 
#         Job(9, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4), 12, timedelta(minutes=10)), 
#         Job(10, datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=3), 5, timedelta(minutes=6))
#     ]
 
#     # stores the maximum deadline that can be associated with a job
#     T = 15
 
#     # schedule jobs and calculate the maximum profit
#     scheduleJobs(jobs, T)