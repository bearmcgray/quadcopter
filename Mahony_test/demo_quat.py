#! /bin/python 
import pygame

from bear.stewart.Matrix import Matrix
from quaternion import Quaternion
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU  import *
from OpenGL.GLUT  import *

try:
    from PIL.Image import open as pil_open
except ImportError, err:
    from Image import open as pil_open
from math import atan2,sin,cos,sqrt,acos

import serial
from serial.tools import list_ports

from madgwick import Madgwick

from ArcBall import *

__cube_div = 2.
ax=146./100./__cube_div
ay=51./100./__cube_div
az=102./100./__cube_div
Dt  = 1/50.

#~ lp_name = "haired.dat"
lp_name = None

verticies = (
##  x  y    z
    Matrix( [[ ax,  ay,  az]]).Transpose(),
    Matrix( [[-ax,  ay,  az]]).Transpose(),
    Matrix( [[-ax,  ay, -az]]).Transpose(),
    Matrix( [[ ax,  ay, -az]]).Transpose(),

    Matrix( [[ ax,  -ay,  az]]).Transpose(),
    Matrix( [[-ax,  -ay,  az]]).Transpose(),
    Matrix( [[-ax,  -ay, -az]]).Transpose(),
    Matrix( [[ ax,  -ay, -az]]).Transpose()
    )

texture=(
	(0,0),(0,1),(1,1),(1,0)
)

sides = (
	##top
	(1,2,3,0),

	##bot
	(6,5,4,7),

	##front
	(5,1,0,4),

	##back
	(7,3,2,6),

	##right
	(4,0,3,7),

	##left
	(6,2,1,5)
)

def rxm(a):
	return Matrix([
	[1,       0,        0 ],
	[0, cos(a), -sin(a)],
	[0, sin(a),  cos(a)]
])

def rym(a):
	return Matrix([
	[ cos(a), 0,  sin(a)],
	[       0,   1,       0 ],
	[-sin(a),  0, cos(a)]
])

def  rzm(a):
	return Matrix([
	[cos(a), -sin(a), 0],
	[sin(a),  cos(a), 0],
	[     0 ,         0 , 1]
])

class kalman_tkj_fast:
	def __init__(self,dt):
		self.__x0 = 0
		self.__x1 = 0

		self.__dt = dt

		self.__F00 = 1
		self.__F01 = -self.__dt
		self.__F10 =  0
		self.__F11 = 1

		self.__B0 = self.__dt
		self.__B1 = 0

		self.__Q00 = 0.05*self.__dt
		self.__Q01 = 0
		self.__Q10 = 0
		self.__Q11 = 0.33*self.__dt

		self.__H0 = 1
		self.__H1 = 0

		self.__P00 = 0
		self.__P01 = 0
		self.__P10 = 0
		self.__P11 = 0

		self.__R = 0.04

	def step(self,ang,vel):

		#~ xkkm = self.__F.DotMul(self.__x).Add(self.__B.ConstMul(vel))
		self.__x0 +=  self.__dt*(vel-self.__x1)

		#~ self.__P = self.__F.DotMul(self.__P).DotMul(self.__F.Transpose()).Add(self.__Q)
		self.__P00 += self.__dt*(self.__dt*self.__P11-self.__P10-self.__P01+self.__Q00)
		self.__P01 -= self.__dt*self.__P11
		self.__P10 -= self.__dt*self.__P11
		self.__P11 += self.__dt*self.__Q11

		#~ yk = ang - self.__H.DotMul(xkkm).Det()
		yk = ang - self.__x0

		#~ Sk = self.__H.DotMul(self.__P).DotMul(self.__H.Transpose()).Add(self.__R)
		Sk = self.__P00+self.__R

		#~ Kk = self.__P.DotMul(self.__H.Transpose()).ConstMul(1./Sk.At(0,0))
		K0 = self.__P00/Sk
		K1 = self.__P10/Sk

		#~ self.__x = xkkm.Add(Kk.ConstMul(yk))
		self.__x0 += K0*yk
		self.__x1 += K1*yk

		#~ self.__P = (Matrix.I(2).Sub(Kk.DotMul(self.__H))).DotMul(self.__P)
		self.__P00 -= K0*self.__P00
		self.__P01 -= K0*self.__P01
		self.__P10 -= K1*self.__P00
		self.__P11 -= K1*self.__P01

		return self.__x0

	def Get(self):
		return self.__x0

class kalman_tkj:
	def __init__(self,dt):
		self.__x = Matrix("0;0")
		self.__dt = dt
		self.__F=Matrix([[1,-self.__dt],[ 0, 1]])
		self.__B=Matrix([[self.__dt],[0]])
		self.__Q=Matrix([[0.005, 0],[0, 0.033]]).ConstMul(self.__dt)
		self.__H=Matrix([[1, 0]])
		self.__P=Matrix([[0, 0],[0, 0]])
		self.__R = Matrix([[0.4]])

	def step(self,ang,vel):
		#~ print ang
		xkkm = self.__F.DotMul(self.__x).Add(self.__B.ConstMul(vel))
		self.__P = self.__F.DotMul(self.__P).DotMul(self.__F.Transpose()).Add(self.__Q)
		yk = ang - self.__H.DotMul(xkkm).Det()
		Sk = self.__H.DotMul(self.__P).DotMul(self.__H.Transpose()).Add(self.__R)
		Kk = self.__P.DotMul(self.__H.Transpose()).ConstMul(1./Sk.At(0,0))
		self.__x = xkkm.Add(Kk.ConstMul(yk))
		self.__P = (Matrix.I(2).Sub(Kk.DotMul(self.__H))).DotMul(self.__P)
		return self.__x.At(0,0)

	def Get(self):
		return self.__x.At(0,0)

class compl:
	def __init__(self,a,dt):
		self.__a = a
		self.__dt = dt
		self.__data=0
	def step(self,ang,gyr):
		self.__data  = (self.__data+gyr*self.__dt)*(self.__a)+(ang)*(1-self.__a)
		return self.__data
	def Get(self):
		return self.__data

def loadImage( imageName ):
	"""Load an image file as a 2D texture using PIL"""
	im = pil_open(imageName)
	try:
	    ix, iy, image = im.size[0], im.size[1], im.tostring("raw", "RGBA", 0, -1)
	except SystemError:
	    ix, iy, image = im.size[0], im.size[1], im.tostring("raw", "RGBX", 0, -1)
	ID = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D, ID)
	glPixelStorei(GL_UNPACK_ALIGNMENT,1)
	glTexImage2D(
	    GL_TEXTURE_2D, 0, 3, ix, iy, 0,
	    GL_RGBA, GL_UNSIGNED_BYTE, image
	)
	return ID

def setupTexture( ID ):
	"""Render-time texture environment setup"""
	glEnable(GL_TEXTURE_2D)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
	glBindTexture(GL_TEXTURE_2D, ID)

def CubeT(v,t):
	for (tos,side) in zip(t,sides):
		setupTexture(tos)
		glBegin(GL_QUADS)
		for (tex,vertex) in zip(texture,side):
			glTexCoord2f(tex[0],tex[1])
			glVertex3f(v[vertex].At(0,0), v[vertex].At(1,0), v[vertex].At(2,0))
		glEnd()
	glDisable(GL_TEXTURE_2D)

def axis(l):
	vector(l,0,0,0,1,1)
	vector(0,l,0,1,0,1)
	vector(0,0,l,1,143.0/255.0,63.0/255.0)

def mod(v):
	return (v[0]*v[0]+v[1]*v[1]+v[2]*v[2])**(0.5)

def cross(u,v):
	return(	u[1]*v[2] - v[1]*u[2],
				-(u[0]*v[2] - v[0]*u[2]) ,
				u[0]*v[1] - v[0]*u[1]
				)

def dot(u,v):
	return u[0]*v[0]+u[1]*v[1]+u[2]*v[2]

def anglev(u,v):
	return acos( dot(u,v) / (mod(u)*mod(v)) )

def vector(x,y,z,c1,c2,c3):
	l = mod((x,y,z))
	vref=(0,0,1)
	raxis = cross(vref,(x,y,z))
	ang = anglev(vref,(x,y,z))

	quadratic = gluNewQuadric()
	glColor4f(c1,c2,c3,0.0)
	glPushMatrix()

	glRotatef(ang*57.3,raxis[0],raxis[1],raxis[2])

	gluCylinder(quadratic, 0.01, 0.01, l*0.9, 15, 2)
	glTranslatef(0,0,l*0.9)
	gluCylinder(quadratic, 0.03, 0.0, l*0.1, 15, 2)
	gluDeleteQuadric(quadratic)
	glPopMatrix()

class logpoints:

	def __init__(self,name):
		self.__eps = 3
		if name:
			self.__destfile  = open(name,"wb")
			self.__destfile.write("gyrox gyroy gyroz accelx accely accelz yaw pitch roll\n")
		else:
			self.__destfile = None

	def Add(self,gx,gy,gz,ax,ay,az,yaw,pitch,roll):
		if self.__destfile is not None:
			gxs = str(round(gx,self.__eps))+" "
			gys = str(round(gy,self.__eps))+" "
			gzs = str(round(gz,self.__eps))+" "

			axs = str(round(ax,3))+" "
			ays = str(round(ay,self.__eps))+" "
			azs = str(round(az,self.__eps))+" "

			#~ mxs = str(round(mx,3))+" "
			#~ mys = str(round(my,3))+" "
			#~ mzs = str(round(mz,3))+" "

			eys =  str(round(yaw,self.__eps))+" "
			eps = str(round(pitch,self.__eps))+" "
			ers = str(round(roll,self.__eps))+" "
			self.__destfile.write(gxs+gys+gzs+axs+ays+azs+eys+eps+ers+"\n")

	def __del__(self):
		if self.__destfile is not None:
			self.__destfile.close()

class mouse_hndl:
	__LEFT_DOWN = 1
	__MIDDLE_DOWN = 2
	__RIGHT_DOWN = 3

	__LEFT_UP = 4
	__MIDDLE_UP = 5
	__RIGHT_UP = 6

	def __init__(self,pg,xx,yy):
		self.__b1 = 0
		self.__b2 = 0
		self.__b3 = 0
		self.__pg = pg
		self.__dx = 0
		self.__dy = 0
		self.__down_flag = 0

		self.g_LastRot = Matrix3fSetIdentity ()							# // Reset Rotation
		self.g_ThisRot = Matrix3fSetIdentity ()							# // Reset Rotation
		self.g_Transform = Matrix4fT ()							# // Reset Rotation
		self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)	# // Reset Rotation
		self.g_ArcBall = ArcBallT (xx, yy)
		self.z = 5

	def __mouse_click(self,p):
		(cursor_x, cursor_y) = self.__pg.mouse.get_pos()
		if p == self.__LEFT_DOWN:
#			print "left down"
			self.g_LastRot = copy.copy (self.g_ThisRot)							# // Set Last Static Rotation To Last Dynamic One
			mouse_pt = Point2fT (cursor_x, cursor_y)
			self.g_ArcBall.click (mouse_pt)								# // Update Start Vector And Prepare For Dragging
		elif p == self.__MIDDLE_DOWN:
			print "middle down"
		elif p == self.__RIGHT_DOWN:
#			print "right down"
			self.g_LastRot = Matrix3fSetIdentity ()							# // Reset Rotation
			self.g_ThisRot = Matrix3fSetIdentity ()							# // Reset Rotation
			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)	# // Reset Rotation
			self.z = 5
		elif p == self.__LEFT_UP:
#			print "left up"
			self.g_LastRot = copy.copy (self.g_ThisRot)							# // Set Last Static Rotation To Last Dynamic One
		elif p == self.__MIDDLE_UP:
			print "middle up"
		elif p == self.__RIGHT_UP:
			#~ print "right up"
			pass

	def __mouse_drag(self):
#		print "mouse drag x:",self.__x," y:",self.__y
		(cursor_x, cursor_y) = self.__pg.mouse.get_pos()
		if ( self.__down_flag&(1<<0) ):
			mouse_pt = Point2fT (cursor_x, cursor_y)
			ThisQuat = self.g_ArcBall.drag (mouse_pt)						# // Update End Vector And Get Rotation As Quaternion
			self.g_ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)		# // Convert Quaternion Into Matrix3fT
			# Use correct Linear Algebra matrix multiplication C = A * B
			self.g_ThisRot = Matrix3fMulMatrix3f (self.g_LastRot, self.g_ThisRot)		# // Accumulate Last Rotation Into This One
			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)	# // Set Our Final Transform's Rotation From This One
			#~ print self.g_LastRot

	def zoom(self,p):
		self.z += p
		if self.z>10:
			self.z = 10
		if self.z<2:
			self.z = 2

	def predif(self,p):
		self.g_LastRot = Matrix3fSetIdentity()
		self.g_ThisRot = Matrix3fSetIdentity()
		self.g_Transform = Matrix4fT()

		if p == 0x31:
			self.g_ThisRot = Matrix3fSetRotationFromQuat4f([-0.707, 0., 0., 0.707])
			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)

		elif p == 0x32:
			self.g_LastRot = Matrix3fSetRotationFromQuat4f([-0.707, 0., 0., 0.707])
			self.g_ThisRot = Matrix3fSetRotationFromQuat4f([0.0, 0.707, 0., 0.707])
			self.g_ThisRot = Matrix3fMulMatrix3f (self.g_LastRot, self.g_ThisRot)

			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot)

		elif p == 0x33:
			self.g_ThisRot = Matrix3fSetRotationFromQuat4f([0., 0., 0.707, 0.707])
			self.g_Transform = Matrix4fSetRotationFromMatrix3f (self.g_Transform, self.g_ThisRot);


	def update(self):
		(b1,b2,b3) = self.__pg.mouse.get_pressed()
		down_flag = 0
		if (self.__b1 == 0 and b1 == 1):
			self.__mouse_click(self.__LEFT_DOWN)
		if (self.__b2 == 0 and b2 == 1):
			self.__mouse_click(self.__MIDDLE_DOWN)
		if(self.__b3 == 0 and b3 == 1):
			self.__mouse_click(self.__RIGHT_DOWN)

		down_flag =  (b1<<0) | (b2<<1) | (b3<<2)

		if (self.__b1 == 1 and b1 == 0):
			self.__mouse_click(self.__LEFT_UP)
		if (self.__b2 == 1 and b2 == 0):
			self.__mouse_click(self.__MIDDLE_UP)
		if(self.__b3 == 1 and b3 == 0):
			self.__mouse_click(self.__RIGHT_UP)

		if down_flag and self.__down_flag == 0:
			(x,y) = self.__pg.mouse.get_rel()
			self.__x = x
			self.__y = y
		else:
			if down_flag:
				(x,y) = self.__pg.mouse.get_rel()
				if x or y:
					self.__x = x
					self.__y = y
					self.__mouse_drag()

		self.__down_flag = down_flag

		self.__b1 = b1
		self.__b2 = b2
		self.__b3 = b3

def findPort(part_of_name):
	for i in list_ports.comports():
		print i
		if i[1].find(part_of_name)>=0:
			return i[0]
		
	return None
	
def main():
	pygame.init()
	display = (600,600)
	pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
	glEnable     ( GL_DEPTH_TEST )
	glClearColor(0,0.5,0.5,0)

	TopID = loadImage('data\\TopInv.png')
	BotID = loadImage('data\\BottomInv.png')

	FrontID = loadImage('data\\FrontInv.png')
	BackID = loadImage('data\\BackInv.png')

	LeftID = loadImage('data\\LeftInv.png')
	RightID = loadImage('data\\RightInv.png')

	com = serial.Serial()
	com.baudrate=115200
	com.timeout=0.001
	#~ com.port = "com7"
	com.port = findPort("CP210x")
	try:
		com.open()
	except:
		print "failed to open port",com.port
		com = None
	if com is not None:
		com.flush()
		com.flushInput()

	log = logpoints(lp_name)

	l = ""
	#~ pc = compl(0.97, Dt)
	#~ pc = kalman_tkj(Dt)
	#~ pc = kalman_tkj_fast(Dt)
	#~ rc = compl(0.97, Dt)
	#~ rc = kalman_tkj(Dt)
	#~ rc = kalman_tkj_fast(Dt)
	#~ yc = compl(1, Dt)

	m = Madgwick(Dt,0.33)
	gc = 1./57.3/16.4
	ac = 1./32768.*9.8*4
	rad2deg = 180.0/3.14159265
	X=Y=Z = 0.0
	VX=VY=VZ = 0.0
	mou = mouse_hndl(pygame,display[0],display[1])

	while True:
		#~ l+=com.read(10)
		glPushMatrix()##0
		gluPerspective(60, (display[0]/display[1]), 0.1, 15.0)
		glTranslatef(0,0,-mou.z)
		glMultMatrixf(mou.g_Transform)
		#~ print mou.g_Transform

		if com is not None:
			btr = com.inWaiting()
			if btr:
				l+=com.read(btr)
		else:
			l += "\r0,0,8192,0,0,0"
		b = l.find("\r")
		e = l[b+1:].find("\r")

		if b>=0 and e>=0:
			try:
				a = [float(i) for i in l[b+1:b+e+1].split(",")]
			except:
				print "l",l[b+1:b+e+1]
			if len(l)>100:
				l=""
				if com is not None:
					com.flush()
					com.flushInput()
			else:
				l=l[b+e+1:]
			if len(a)!=6:
				print" continue"
				continue

			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

			#~ pitch = atan2(a[0], a[2]);
			#~ roll  = atan2( a[1], sqrt( a[0] * a[0] + a[2] * a[2]));

			#~ pc.step(pitch*57.3, -a[4]/32.8/4)
			#~ rc.step(roll*57.3, a[3]/32.8/4)
			#~ yc.step(0, a[5]/32.8/4)

			#~ gx = a[3]*gc
			#~ gy = a[4]*gc
			#~ gz = a[5]*gc

			#~ ax = a[0]*ac
			#~ ay = a[1]*ac
			#~ az = a[2]*ac
			#~ print round((ax*ax+ay*ay+az*az)**(0.5),2)

			#~ mx = -(a[6]+130)
			#~ my = (a[7]-100)##must be negativ but now wrong c code in hmc5883.c line 81
			#~ mz = -(a[8]-10)##must be positive but now wrong c code in hmc5883.c line 82
			#~ mx = -(a[6]+230)
			#~ my = -(a[7]+100)##must be negativ but now wrong c code in hmc5883.c line 81
			#~ mz = (a[8]+100)##must be positive but now wrong c code in hmc5883.c line 82
			
			
			#~ m.MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz)
			#~ print gx, gy, gz, ax, ay, az, mx, my, mz
			#~ m.MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az)
			

			qu = [i/1000. for i in a[:4]]
			print qu
			glPushMatrix()##1

#			glRotatef(-90,1,0,0)

			#~ glTranslatef(X,Y,Z)
			glRotatef(2*acos(qu[0])*rad2deg,qu[1],qu[2],qu[3])
			glRotatef(90,1,0,0)

			CubeT(verticies,[TopID,BotID,FrontID,BackID,RightID,LeftID])

			#~ acc = Quaternion(0, ax, ay, az)
			#~ mag = Quaternion(0, mx/512., my/512., mz/512.)

			
			#~ mq = Quaternion(qu[0],qu[1],qu[2],qu[3])

			#~ accEarth = mq*acc*mq.Conj()
			#~ magEarth = mq*mag*mq.Conj()


			glPopMatrix()##~1

			#~ glPushMatrix()##2
			## earth magnit field vector
			#~ vector(magEarth.At(1), magEarth.At(2), magEarth.At(3), 1,0.75,0.5)##magneto
			#~ vector(magEarth.At(1), magEarth.At(2),              0, 1,0.75,0.5)##magneto
			## accceleration vector
			#~ grav = Quaternion(0. ,0., 0., 9.8 )
			#~ accEarth = accEarth-grav
#			vector(accEarth.At(1),accEarth.At(2),accEarth.At(3),1,1,0)##acceleration in earth
			#~ vector(0,0,accEarth.At(3),1,1,0)##acceleration in earth

			#~ glPopMatrix()##~2

			#~ glPushMatrix()##3
#			glRotatef(-90,0,0,1)
			#~ glRotatef(-90,1,0,0)

			#~ print VZ,Z
			#~ VX+=-accEarth.At(1)*Dt*0
			#~ VY+=-accEarth.At(2)*Dt*0
			#~ VZ+=-accEarth.At(3)*Dt*0

			#~ if accEarth.Mod()<0.5:
				#~ VX=VY=VZ = 0.0

			#~ X+=VX*Dt
			#~ Y+=VY*Dt
			#~ Z+=VZ*Dt

			#~ glTranslatef(X,Y,Z)
			#~ print VX,VY,VZ
			#~ if abs(X)>5 or abs(Y)>5 or abs(Z)>5:
				#~ X=Y=Z=0.0
				#~ VX=VY=VZ = 0.0
			#~ CubeT(verticies,[TopID,BotID,FrontID,BackID,RightID,LeftID])
			#~ vector(grav.At(1),grav.At(2),grav.At(3),1,1,1)
			#~ glPopMatrix()##~3
			##v = [ rym(yc.Get()/57.3).DotMul(rxm(rc.Get()/57.3)).DotMul(rzm(pc.Get()/57.3)).DotMul(i) for i in verticies]
			m.Set(qu[0],qu[1],qu[2],qu[3])
			e = m.GetEuler()

			log.Add(0,0,0,0,0,0,e["yaw"]*rad2deg,e["pitch"]*rad2deg,e["roll"]*rad2deg)

			#~ print round(e["yaw"]*rad2deg,2)
			#~ print round(e["yaw"]*rad2deg,2),round(e["pitch"]*rad2deg,2),round(e["roll"]*rad2deg,2)
			axis(1.5)
			pygame.display.flip()

		mou.update()
		glPopMatrix()##~0

		for event in pygame.event.get():
			if event.type == 5:##mouse down
				if event.button == 5:
					mou.zoom(0.3)
				elif event.button == 4:
					mou.zoom(-0.3)
			elif event.type == pygame.QUIT:
				pygame.quit()
				quit()

			elif event.type == 2:
				mou.predif(event.key)
				#~ print hex(event.key)

if __name__=="__main__":
	main()