from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence
from panda3d.core import *
from direct.gui.DirectGui import *

from TelemetryService import *

class MyApp(ShowBase):


	# declare member variables
	tmService = TM_Service(
				port='/dev/ttyUSB0',
				baudrate=57600,
				timeout=0.1)
	
		

	def __init__(self):

		ShowBase.__init__(self)

		# add ambient light for room
		ambientLight = AmbientLight('ambientLight')
		ambientLight.setColor(Vec4(1,1,1,1))
		ambientLightNP = self.render.attachNewNode(ambientLight)
		self.render.setLight(ambientLightNP)

		# disable mouse control (allow camera position in code)
		self.disableMouse()

		# load the environment model
		self.scene = self.loader.loadModel("/home/chris/Documents/opengl/python/models/island/island.egg")

		# reparent the model to render
		self.scene.reparentTo(self.render)

		# apply scale and position transforms on the model
		scaling = 10
		self.scene.setScale(scaling*2,scaling*2,scaling)
		self.scene.setPos(0,0,-40)
		self.scene.setHpr(135,0,0)

		# setup camera positioning
		zoomFactor = 0.15
		self.camera.setPos(25/zoomFactor,25/zoomFactor,40/zoomFactor)
		self.camera.setHpr(135,-45,0)

		# load sky model
		self.sky = self.loader.loadModel("/home/chris/Documents/opengl/python/models/blueSky/blue_sky_sphere.egg")
		self.sky.setScale(1,1,1)
		self.sky.setPos(0,0,0)
		self.sky.reparentTo(self.render)
		
		# # add camera task
		# self.taskMgr.add(self.spinCameraTask,"SpinCameraTask")

		# load airplane
		self.planeModel = self.loader.loadModel("/home/chris/Documents/opengl/python/models/jet/jet.egg")
		self.planeModel.setPos(0,0,40)
		self.planeModel.setScale(0.4,0.4,0.4)
		self.planeModel.setHpr(180,0,0)
		self.planeModel.reparentTo(self.render)

		# add plane attitude update task
		self.taskMgr.add(self.redrawAirplane,"RedrawAirplane")

		# add camera buttons
		self.cameraPlaneFrameButton = DirectButton(text=("PLANE\nFRAME","PLANE\nFRAME","PLANE\nFRAME?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(-1.3,0,-0.9),command=self.setPlaneFrame)
		self.cameraSceneFrameButton = DirectButton(text=("SCENE\nFRAME","SCENE\nFRAME","SCENE\nFRAME?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(-1.1,0,-0.9),command=self.setSceneFrame)

		self.cameraIsoButton = DirectButton(text=("ISO","ISO","ISO?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(-0.9,0,-0.9),command=self.setIsoView,extraArgs=[zoomFactor])
		

		self.cameraDefaultButton = DirectButton(text=("Default","Default","Default?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(-0.7,0,-0.9),command=self.setDefaultView,extraArgs=[zoomFactor])

		self.cameraFrontButton = DirectButton(text=("Front","Front","Front?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(-0.2,0,-0.9),command=self.setFrontView,extraArgs=[zoomFactor])
		self.cameraBackButton = DirectButton(text=("Back","Back","Back?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(0,0,-0.9),command=self.setBackView,extraArgs=[zoomFactor])

		self.cameraTopButton = DirectButton(text=("Top","Top","Top?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(0.2,0,-0.9),command=self.setTopView,extraArgs=[zoomFactor])
		self.cameraBottomButton = DirectButton(text=("Bottom","Bottom","Bottom?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(0.4,0,-0.9),command=self.setBottomView,extraArgs=[zoomFactor])

		self.cameraRightButton = DirectButton(text=("Right","Right","Right?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(0.6,0,-0.9),command=self.setRightView,extraArgs=[zoomFactor])
		self.cameraLeftButton = DirectButton(text=("Left","Left","Left?","disabled"),text_bg=Vec4(0,0,0,1),text_fg=Vec4(1,0,0,1),scale=0.05,relief=None,pos=Vec3(0.8,0,-0.9),command=self.setLeftView,extraArgs=[zoomFactor])


		self.setDefaultView(zoomFactor)


	def setTopView(self,zoomFactor):

		self.camera.setPos(0,0,self.planeModel.getZ()*2/zoomFactor)
		self.camera.setHpr(0,-90,0)
		return Task.cont

	def setBottomView(self,zoomFactor):

		self.camera.setPos(0,0,-self.planeModel.getZ()*2/zoomFactor)
		self.camera.setHpr(0,90,0)
		return Task.cont

	def setRightView(self,zoomFactor):

		self.camera.setPos(40/zoomFactor,0,self.planeModel.getZ())
		self.camera.setHpr(90,0,0)
		return Task.cont
	
	def setLeftView(self,zoomFactor):

		self.camera.setPos(-40/zoomFactor,0,self.planeModel.getZ())
		self.camera.setHpr(-90,0,0)
		return Task.cont

	def setBackView(self,zoomFactor):

		self.camera.setPos(0,-40/zoomFactor,self.planeModel.getZ())
		self.camera.setHpr(0,0,0)
		return Task.cont

	def setFrontView(self,zoomFactor):

		self.camera.setPos(0,40/zoomFactor,self.planeModel.getZ())
		self.camera.setHpr(180,0,0)
		return Task.cont








	def setIsoView(self,zoomFactor):
		
		self.camera.setPos(25/zoomFactor,25/zoomFactor,40/zoomFactor)
		self.camera.setHpr(135,-45,0)
		return Task.cont

	def setDefaultView(self,zoomFactor):
		
		self.camera.setPos(0,40/zoomFactor,40/zoomFactor)
		self.camera.setHpr(180,-45,0)
		return Task.cont

	def setPlaneFrame(self):
	
		self.camera.reparentTo(self.planeModel)
		return Task.cont

	def setSceneFrame(self):
	
		self.camera.reparentTo(self.render)
		return Task.cont




	def redrawAirplane(self,task):
		
		# fetch airplane attitude
		#roll = 45.0*cos(task.time*6.0*pi/180.0)
		#pitch = 45.0*sin(task.time*6.0*pi/180.0)
		#yaw = task.time*6.0

		# update TM
		tmUpdated = self.tmService.updateTM()
		if tmUpdated:
			
			print ''
			print ''
			print ''
			print ''
			print "Quat 1: ",self.tmService.TM_Data.q1
			print "Quat 2: ",self.tmService.TM_Data.q2
			print "Quat 3: ",self.tmService.TM_Data.q3
			print "Quat 4: ",self.tmService.TM_Data.q4
			print ''
			print "Delta Time (sec): ",self.tmService.TM_Data.deltaTimeSec
			print ''
			print "Rate X (deg/sec): ",self.tmService.TM_Data.gx
			print "Rate Y (deg/sec): ",self.tmService.TM_Data.gy
			print "Rate Z (deg/sec): ",self.tmService.TM_Data.gz
			print ''
			print "Accel X (m/s^2): ",self.tmService.TM_Data.ax
			print "Accel Y (m/s^2): ",self.tmService.TM_Data.ay
			print "Accel Z (m/s^2): ",self.tmService.TM_Data.az

			# compute the body to model transformation
			quatBodyToModel = Quat(math.sqrt(2)/2.0,0.0,0.0,-math.sqrt(2)/2.0) # 90 degree z-rotation

			# compute model quaternion
			quatPlaneEciToBody = Quat(
							self.tmService.TM_Data.q4,
							self.tmService.TM_Data.q1,
							-self.tmService.TM_Data.q2,
							-self.tmService.TM_Data.q3
						)

			self.planeModel.setQuat(quatBodyToModel*quatPlaneEciToBody)

		# readjust plane attitude
		#self.planeModel.setHpr(yaw,pitch,roll)

		# return (indicate execution)
		return Task.cont


		


	def spinCameraTask(self,task):
		
		angleDegrees = task.time*6.0
		angleRadians = angleDegrees*(pi/180.0)
		self.camera.setPos(20.0*sin(angleRadians),-20.0*cos(angleRadians),3)
		self.camera.setHpr(angleDegrees,0,0)
		return Task.cont



app = MyApp()
app.run()


