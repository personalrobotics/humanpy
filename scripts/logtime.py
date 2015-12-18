#!/usr/bin/env python

import rospy
import numpy as np

class RingBuffer():
	"A X-D ring buffer using numpy arrays"
	def __init__(self, length, width ):
		self.width = width
		self.length = length
		self.data = np.zeros((length,width+1), dtype='f')
		self.idstr= np.zeros((length,254), dtype='c')
		self.index = 0
		self.ring  = 0

	def insert(self, x, label):
		if len(x) != self.width:
			raise Exception("Error RingBuffer().insert() input dimension")
			
		"adds array x to ring buffer"
		x_index = (self.index + np.arange(1)) % self.length
		self.ring = self.ring + (x_index+1) / self.length
		self.data[x_index,0] = len(label)
		self.data[x_index,1:] = x
		self.idstr[x_index,:len(label)] = label
		self.index = x_index[-1] + 1

	def get(self, idx = None):
		
		if idx == None:
			"Returns the first-in-first-out data in the ring buffer"
			if self.ring == 0:
				idx = np.arange(self.index)
			else:
				idx = (self.index + np.arange(self.length)) % self.length
				
			labels = [ self.idstr[ jdx, :self.data[jdx,0]  ].tostring() for jdx in idx ]
			return labels, self.data[idx,1:]
		else:
			"Returns the first-in-first-out data in the ring buffer"
			labels = self.idstr[ idx, :self.data[idx,0]  ].tostring()
			return labels, self.data[idx,1:]
			

class TimeLog:
	
	def __init__(self,length):
		self.time_buffer = RingBuffer( length, 5 )	
		self.start_time = None
		self.end_time = None
		
	def set_absolute_time(self, label ):
		t = rospy.get_rostime()
		self.time_buffer.insert( [ t.secs,t.nsecs ,0.,0.,0.], label ) 

	def tic(self):
		self.start_time = rospy.get_rostime()

	def toc(self,label):
		self.end_time = rospy.get_rostime()
		if self.start_time == None:
			print "Toc() without Tic(), errror"
			return
			
		dt = self.end_time - self.start_time
		self.time_buffer.insert( [ self.start_time.secs,self.start_time.nsecs, self.end_time.secs,self.end_time.nsecs,dt.to_sec()], label ) 
		self.start_time = None
		self.end_time = None
		
	def get(self, idx = None):
		return self.time_buffer.get(idx)

if __name__ == "__main__":

    rospy.init_node('test')


	#Test of the RingBUffer
    ringlen = 10
    ringbuff = RingBuffer(ringlen, 3)
    for i in range(100):
        ringbuff.insert([1.+i,2.*i,3.*i],"ciao"+str(i)) # write
        
    for i in range(10):
		label,val = ringbuff.get(i) #read
		print label, ": ", val
		
    #Test of the TimeLog
    timelogger = TimeLog(ringlen)
    for i in range(1000):
		if (i % 7) == 0:
			#print "timelogger.set_absolute_time	()"+str(i)
			timelogger.set_absolute_time("ABSOLUTE")
		
		if ( i % 2 ) == 0:
			#print "timelogger.Tic()"
			timelogger.tic()
		elif ((i-1)) % 2 == 0:	
			#print "timelogger.Toc()"
			timelogger.toc("NEW DELTA"+str(i))
		
		rospy.sleep(0.01)
        
    for i in range(10):
		label,val = timelogger.get(i) #read
		print label, ": " 
		print "      start: ", val[0]  + val[1]/ 1.e9 
		if label != "ABSOLUTE":
			print "      end: ", val[2] + val[3]/ 1.e9 
			print "      dt: ", val[4] 
		else:
			print " "

