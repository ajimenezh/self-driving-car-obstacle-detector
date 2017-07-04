import demo
from Queue import Queue
from Queue import Empty
from threading import Thread
from sets import Set
from utils.timer import Timer
from time import sleep
from threading import Thread, Lock

mutex = Lock()

TESTING = True

if TESTING:
    file = open("boxes_ford02_v2.txt")
    lines = file.readlines()
    i = 0
    
    boxes = []
    times = []
    
    while i < len(lines):
        t = int(lines[i][:-1])
        i += 1
        times.append(t)
        
        n = int(lines[i][:-1])
        i += 1
        
        if n > 0:
            s = lines[i][:-1].split(" ")[:-1]
            v = [float(s[j]) for j in range(4)]
            boxes.append(v)
        
            i += 1
        else:
            boxes.append([0.0,  0.0,  0.0,  0.0])

def work(q, obj):

  det = demo.Detector()

  while True:
	try:
		file = q.get()
		
		if True or not file in obj.cache_camera:
			obj.cache_camera.add(file)

			mutex.acquire()
			try:
				obj.working = True
			finally:
				mutex.release()
			
			s = file.split("/")
			s = s[-1][:-4]
			
			if TESTING:
				v = [0.0,  0.0,  0.0,  0.0]
				for i in range(len(times)):
					if times[i] == int(s):
					    v = boxes[i]
					    v[1] -= 400
					    v[3] -= 400
					    break
					    
				res = [[(v, "car")]]	
				print res
				sleep(0.4)
			else:
				res = det.detect(file)

			print "Hello world"

			obj.out.write(s + "\n")
			obj.out.write(str(len(res)) + "\n")
			obj.out.flush()

			if res != None:
				s = file.split("/")
				s = s[-1][:-4]
				
				objs = []
				
				for elem1 in res:
					for elem in elem1:
						if elem[1] == "person" or elem[1] == "car":
							elem[0][1] += 400
							elem[0][3] += 400
							
							objs.append(elem[0])

							for j in range(4):
								obj.out.write(str(elem[0][j]))
								obj.out.write(" ")
							obj.out.write("\n")
							obj.out.flush()
				
				obj.callback(objs, int(s))
				
			mutex.acquire()
			try:
				obj.working = False
			finally:
				mutex.release()

		q.task_done()
	except Empty:
		#Handle empty queue here
		pass

class DetectorHelper:

	def __init__(self, mode, filename):

		if mode == 0:
			return

		self.q = Queue(maxsize=0)
		self.q_lidar = Queue(maxsize=0)
		self.num_threads = 1

		worker = Thread(target=work, args=(self.q,self,))
		worker.setDaemon(True)
		worker.start()

		self.cache_camera = Set()

		self.out = open("boxes_" + filename + ".txt", "w")
		self.out_times = open("times_" + filename + ".txt", "w")

		self.finished = False
		self.working = False

	def detect(self, file):
		if (True or not file in self.cache_camera) and not self.working:
			self.q.put(file)
			s = file.split("/")
			s = s[-1][:-4]
			self.out_times.write(s + "\n")
			self.out_times.flush()

	def setCallback(self, f):
		self.callback = f

if __name__ == '__main__':
	obj = demo.Detector()

	print obj.detect("./camera/1492887008003379578.jpg")
