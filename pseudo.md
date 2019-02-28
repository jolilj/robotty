```
class RTS(ABC)
	def __init__(self, freq):
		self.freq = freq
  	
  	def start():
  		while self.is_running:
  			tic
  			update()
  			toc
  			remaining = 1/self.freq - (toc-tic)
  			if (remaining > 0)
	  			sleep(remaining)
	  		else:
	  			print(WARNING)
  	
  	def stop():
  		self.is_running = False
  	 
	def update():
		# do your shit here
		not implemented
```

```
class ISensor(Object):
	def fetch(self):
```

```
class Robotty(RTS):
	def __init__(self):
		super(100)
		self.lwheelencoder = WheelEncoder(self.freq)
		self.rwheelencoder = WheelEncoder(self.freq)
		
	def update():	
		# MAYBE: self.lwheelencoder.update()
		
		self.kf.predict()

		lw = lwheelencoder.fetch()
		rw = rwheelencoder.fetch()

		if lw:
			self.kf.update_lw(lw)
		if rw:
			self.kf.update_rw(rw)
			
		...

```

```
class WheelEncoder(ISensor):
	def __init__(self):
		self.omega = 0
		self.tick_per_rot = 20  # TODO: is this correct?????????
		self.ticks_during_frame = 0
		os.register_callback(17, RISING_EDGE, self.tick)
		# GPIO.add_event_detect(17, GPIO.FALLING, callback=self.tick, bouncetime=300)
	
	def tick():
		self.tick++
	
	def update():
		omega = (2 * pi / self.tick_per_rot) * self.tick * self.freq
		
	def fetch():
		ret = self.omega
		
```