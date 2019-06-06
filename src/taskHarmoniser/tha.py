from TaskHarmoniser import TaskHarmoniser
import time
import threading

_FINISH = False

def initDA(th):
	i=0
	while i < 6:
		th.addDA(i)
		th.updatePriority(i,i)
		time.sleep(1)
		i = i+1
		print (i)
def scheduler(th):
	global _FINISH
	while True:
		th.schedule()
		time.sleep(1)
		if _FINISH:
			break
def switcher(th):
	global _FINISH
	while True:
		th.switchDA()
		if _FINISH:
			break

if __name__== "__main__":
	th = TaskHarmoniser()
	initiator = threading.Thread(target = initDA, args=[th])
	initiator.start()
	scheduler = threading.Thread(target = scheduler,  args=[th])
	scheduler.start()
	switcher = threading.Thread(target = switcher,  args=[th])
	switcher.start()
	while True:
		time.sleep(1)
		if not initiator.is_alive():
			global _FINISH
			_FINISH = True
			break

	initiator.join()   
	scheduler.join()   
	switcher.join()   
	print("END")
    	