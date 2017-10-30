#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

from framework import *
import math
from math import sin,cos,sqrt
import ConfigParser
#~ from random import random



class filter:
	def __init__(self,a):
		self.__a = float(a)
		self.__p = 0.0
		
	def __call__(self,e):
		self.__p = self.__p*(1-self.__a) + e*self.__a 
		return self.__p

class graph:
	def __init__(self,renderer,x0,y0,w,h):
		self.__x0 = x0
		#~ self.__k = k
		self.__y0 = y0
		self.__w = w
		self.__h = h
		self.__renderer = renderer
		self.__data=[(0.00,0.00,0.00)]*self.__w
		#~ print self.__data
		self.__c = int(self.__y0+self.__h/2.)
		
		self.__c1 = int(self.__y0+self.__h/3.)
		self.__c2 = int(self.__y0+self.__h*2./3.)
		self.__c3 = int(self.__y0+self.__h)
		
		self.__color=[
			b2Color(0.5,0.25,0.75),
			b2Color(0.75,0.25,0.5),
			b2Color(0.5,0.75,0.25)
		]
	
	def __autoScale(self,lst,idx):
		mx = max(map(lambda i: i[idx],lst))
		mi = min(map(lambda i: i[idx],lst))
		if abs(mx)>abs(mi):
			m = abs(mx)
		else:
			m = abs(mi)
		if m==0:
			m=1.
		if m < 1./50:
			m=1./50
		#~ return self.__h/(2.*3)/m
		return self.__h/2./m
		
	def UpdateGraph(self,p1,p2,p3):
		del self.__data[0]
		#~ self.__data.append( (self.__c1-int(self.__k*p1),self.__c2-int(self.__k*p2),self.__c3-int(self.__k*p3)) )
		self.__data.append((p1,p2,p3))
		k0=self.__autoScale(self.__data,0)
		k1=self.__autoScale(self.__data,1)
		k2=self.__autoScale(self.__data,2)
		mkoef = [k0,k1,k2]
		
		for i,e in enumerate(self.__data):
			for j,k,c in zip(e,mkoef,self.__color):
				self.__renderer.DrawPoint((self.__x0+i,int(j*k+self.__c)),0,c)
		
		
class config:
	
	def __init__(self,name):
		self.__name = name
		self.__default = {"kp":0.0,"ki":0.0,"kd":0.0,"dt":1.0/60.,"min":-math.pi/8,"max":math.pi/8}
		self.__config = ConfigParser.RawConfigParser()
		
	def Load(self):
		self.__config.read(self.__name)
		
	def Read(self,section_name):
		if section_name=="setup":
			if not self.__config.has_section(section_name):
				self.CreateDefault(section_name,{"configer":0})
			conf=self.__config.getint(section_name,"configer")
			
		else:
			
			if not self.__config.has_section(section_name):
				self.CreateDefault(section_name,self.__default)
			
			kp = self.__config.getfloat(section_name,"kp")
			ki = self.__config.getfloat(section_name,"ki")
			kd = self.__config.getfloat(section_name,"kd")
			dt = self.__config.getfloat(section_name,"dt")
			mi = self.__config.getfloat(section_name,"min")
			mx = self.__config.getfloat(section_name,"max")
			
			return (kp,ki,kd,dt,mi,mx)

	def CreateDefault(self,section_name,defaults):
		try:
			self.__config.add_section(section_name)
		except ConfigParser.DuplicateSectionError:
			pass
			
		for i,v in defaults.iteritems():
			self.__config.set(section_name,i,str(v))

	def Write(self,section_name,kp,ki,kd,dt,mi,mx):
		self.__config.remove_section("DEFAULT")
		
		self.__config.set(section_name, 'kp', str(kp))
		self.__config.set(section_name, 'ki', str(ki))
		self.__config.set(section_name, 'kd', str(kd))
		self.__config.set(section_name, 'dt', str(dt))
		self.__config.set(section_name, 'min', str(mi))
		self.__config.set(section_name, 'max', str(mx))
		
		with open(self.__name, 'wb') as configfile:
			self.__config.write(configfile)		
		
		print "write"

class qcopter:
	body = None
	THRUST_KOEF = 5
	cut = 1
	thrust = 0.0
	x_pid = None
	a_pid = None
	y_pid = None
	
	configer = 1
	# 0 - a_pid; 1 - y_pid; 2 - x_pid 
	def saveConf(self):
		(kp,ki,kd) = self.a_pid.GetCoefs()
		self.__conf.Write("a_pid",kp,ki,kd,self.a_pid.dt,self.a_pid.min,self.a_pid.max)
		(kp,ki,kd) = self.y_pid.GetCoefs()
		self.__conf.Write("y_pid",kp,ki,kd,self.y_pid.dt,self.y_pid.min,self.y_pid.max)
		(kp,ki,kd) = self.x_pid.GetCoefs()
		self.__conf.Write("x_pid",kp,ki,kd,self.x_pid.dt,self.x_pid.min,self.x_pid.max)


	def __init__(self,world,renderer,a,b,x,y,ang,mode=0):
		"""	mode = 0 - x_tgt & y_tgt
			mode = 1 - angle & thrust
		"""
		self.world = world
		self.renderer = renderer
		self.a = a
		self.b = b
		self.x = x
		self.x_tgt = x
		self.y = y
		self.y_tgt = y
		self.ang = ang
		self.mode = mode
		self.tgt = []
		self.thrust_color = b2Color(1,1,1)
		
		self.__f = filter(0.9)
		self.__ftl = filter(0.65)
		self.__ftr = filter(0.65)
		
		self.AddTarget(b2Vec2(self.x,self.y))
		self.NextTarget()
		##self.__g = graph(self.renderer,45,300,10,100,100)		
		
		##self.a_pid = PID(0.07, 0.00001, 0.009, 1./64., 0.5, -0.5)
		##self.y_pid = PID(0.5, 0.01, 0.9, 1./64., 0.3, 0)
		##self.x_pid = PID(0.04, 0.000001, 0.1, 1./64., 3.14/8, -3.14/8)
		
		self.__conf = config("config.cfg")
		self.__conf.Load()
		if mode:
			(kp,ki,kd,dt,mi,mx)=self.__conf.Read("a_pid")
			self.a_pid = PID(kp, ki, kd, dt, mx, mi, graph(self.renderer,300,10,300,100) if self.configer==0 else None)
			(kp,ki,kd,dt,mi,mx)=self.__conf.Read("y_pid")
			self.y_pid = PID(kp, ki, kd, dt, mx, mi, graph(self.renderer,300,10,300,100) if self.configer==1 else None)
			(kp,ki,kd,dt,mi,mx)=self.__conf.Read("x_pid")
			self.x_pid = PID(kp, ki, kd, dt, mx, mi ,graph(self.renderer,300,10,300,100) if self.configer==2 else None)
			self.thrust_color = b2Color(1,0,0.5)
		else:
			self.a_pid = PIDz(2.0, 0.1, 6.0,  0.5, -0.5)
			self.y_pid = PIDz(6.2, 0.2, 17.4, 0.5, 0)
			self.x_pid = PIDz(0.6, 0.02, 22, math.pi/8, -math.pi/8)
			self.thrust_color = b2Color(0,0,1)
		
		rail=b2FixtureDef(
			shape=b2PolygonShape(box=(self.a,self.b)),
			density=0.02,
			friction=0.1)

		body=b2FixtureDef(
			shape=b2PolygonShape(box=(a/3.,a/6.,(0,a/6.),0)),
			density=0.10,
			friction=0.1)
		#~ help(b2PolygonShape)	
		self.body = self.world.CreateDynamicBody(fixtures=[rail,body], position=(self.x, self.y))		
			
	def SetCut(self,cut):
		self.cut = cut
		
	def GetPos(self):
		return self.body.position
	
	def SetTarget(self,p):
		self.x_tgt = p.x
		self.y_tgt = p.y

	def GetVel(self):
		#~ print dir(self.body)
		return sqrt(self.body.linearVelocity.x**2+self.body.linearVelocity.y**2)

	def IsTargetComplete(self):
		tmp = sqrt( (self.x_tgt-self.body.position.x)**2+(self.y_tgt-self.body.position.y)**2 )
		return tmp<2.0
		
	def DrawTarget(self):
		self.renderer.DrawCircle(self.renderer.to_screen(b2Vec2(self.x_tgt,self.y_tgt)),3,b2Color(0.5,1,0.25),1)
		
	def NextTarget(self):
		if len(self.tgt)>0:
			self.SetTarget(self.tgt[0])
			del self.tgt[0]

	def AddTarget(self,p):
		self.tgt.append(p)
		#~ print self.tgt
		
	def DrawTargets(self):
		for i in self.tgt:
			self.renderer.DrawCircle(self.renderer.to_screen(b2Vec2(i.x,i.y)),3,b2Color(0.25,0.5,0.99),1)

	def SetTargetAng(self,a):
		self.a_tgt = a
	
	def Update(self):

		if self.body.angle>math.pi:
			self.body.angle -= math.pi*2
		if self.body.angle<-math.pi:
			self.body.angle += math.pi*2
			
		alpha_rnd =1* b2Random(-math.pi/180.0/2, math.pi/180.0/2)
		h_rnd = 1*b2Random(-0.002,0.002)
		
		cur_angle_with_noise = self.__f(self.body.angle + alpha_rnd)
		#~ print self.x_tgt,self.body.position.x
		
		if self.x_pid:
			pid_ang   = self.x_pid.Update(self.x_tgt, self.body.position.x)
		else:
			pid_ang  = 0
		
		d_torque =  self.a_pid.Update(-pid_ang, cur_angle_with_noise)
		if self.y_pid:
			y_torque =   self.y_pid.Update(self.y_tgt, self.body.position.y+h_rnd)		
		else:
			y_torque = 0.07
		#~ y_torque =   0.07
		#~ print d_torque
		tl = self.__ftl(self.thrust + y_torque - d_torque)
		tr = self.__ftr(self.thrust + y_torque + d_torque)
		
		
		tl*=self.cut
		tr*=self.cut
		
		if tl<0:tl =0
		if tr<0:tr =0
		
		##self.__g.UpdateGraph(tl,tr)
		
		pl = self.body.GetWorldPoint(localPoint=(-self.a, 0))
		pr = self.body.GetWorldPoint(localPoint=( self.a, 0))
		self.body.ApplyForce(b2Vec2(tl*sin(-self.body.angle), tl*cos(-self.body.angle)), pl, True)
		self.body.ApplyForce(b2Vec2(tr*sin(-self.body.angle), tr*cos(-self.body.angle)), pr, True)	
	
		plt = self.body.GetWorldPoint(localPoint=(-self.a, -tl*self.THRUST_KOEF))
		prt = self.body.GetWorldPoint(localPoint=( self.a, -tr*self.THRUST_KOEF))
		
		transform1 = b2Transform()
		transform2 = b2Transform()
		transform1.Set(plt,self.body.angle+math.pi/2)
		transform2.Set(prt,self.body.angle+math.pi/2)
		polygon1 = b2PolygonShape(box=(tl*self.THRUST_KOEF,0.01))
		polygon2 = b2PolygonShape(box=(tr*self.THRUST_KOEF,0.01))

		for shape, transform in [(polygon1, transform1),(polygon2, transform2)]:
			new_verts = [self.renderer.to_screen(transform*v) for v in shape.vertices]
			self.renderer.DrawSolidPolygon(new_verts, self.thrust_color)
		
	def GetErr(self):
		return b2Vec2(self.x_tgt-self.body.position.x, self.y_tgt-self.body.position.y)
		
	def GetTarget(self):
		return b2Vec2(self.x_tgt,self.y_tgt)		
	
	def SetCoefs(self,tgt,kp,ki,kd):
		if tgt==0:
			self.a_pid.SetCoefs(kp,ki,kd)
		elif tgt==1:
			self.y_pid.SetCoefs(kp,ki,kd)
		elif tgt==2:
			self.x_pid.SetCoefs(kp,ki,kd)
			
	def GetCoefs(self,tgt):
		if tgt==0:
			return self.a_pid.GetCoefs()
		elif tgt==1:
			return self.y_pid.GetCoefs()
		elif tgt==2:
			return self.x_pid.GetCoefs()
			

		
class PID_typeC:
	"""http://bestune.50megs.com/typeABC.htm
	"""
	kp = 0.0
	ki = 0.0
	kd = 0.0
	def __init__(self,kp,ki,kd,t,max,min,plot):
		self.SetCoefs(kp,ki,kd)
		self.pv=[0.0,0.0,0.0]
		self.co = 0.0
		self.min = min
		self.max = max
		self.__t = t
		self.__plot = plot
		self.__e = 0
		
	def SetCoefs(self,kp,ki,kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd	
		#~ self.co = 0.0
	
	def GetCoefs(self):
		return (self.kp,self.ki,self.kd)
	
	def Update(self,sp,pv):
		self.__e = float(sp) - float(pv)
		del self.pv[0]
		self.pv.append(pv)
		k = -1## dont look at me
		p_part = -self.kp*(self.pv[k]-self.pv[k-1])
		i_part  = self.ki*self.__e
		d_part = - self.kd*(self.pv[k]-2*self.pv[k-1]+self.pv[k-2])
		
		if self.__plot:
			self.__plot.UpdateGraph(p_part*30,i_part*300,d_part*30)		
		
		delta = p_part + i_part + d_part
		
		self.co += delta
		#~ print "set:",sp,"process:",pv,"err:",e,"delta:",delta,"co:",self.co
		if self.co>self.max:
			self.co = self.max
		if self.co<self.min:
			self.co = self.min
		
		#~ print self.co/3.13*180
		return self.co	
		
	def GetError(self):
		return self.__e
		
class PIDz:
	def __init__(self,kp,ki,kd,max,min):
		self.SetCoefs(kp,ki,kd)
		self.ez=[0.0,0.0,0.0]
		self.max = max
		self.min = min
		self.u = 0.0
	
	def SetCoefs(self,kp,ki,kd):
		self.k1 = kp+ki+kd
		self.k2 = -kp-2.0*kd
		self.k3 = kd
		
	def Update(self,setpoint,y):
		e = float(setpoint) - float(y)
		del self.ez[0]
		self.ez.append(e)
		delta_u = self.k1*self.ez[-1]+self.k2*self.ez[-2]+self.k3*self.ez[-3]
		#~ print "set:",setpoint,"y:",y,"err:",e,"delta:",delta_u,"k1:",self.k1,"k2:",self.k2,"k3:",self.k3
		self.u += delta_u
		
		if self.u>self.max:
			self.u = self.max
		if self.u<self.min:
			self.u = self.min
			
		return self.u

class PID:
	def __init__(self,kp,ki,kd,dt,max,min,plot=None):
		self.KP = kp
		self.KI = ki
		self.KD = kd
		self.dt = dt
		self.min = min
		self.max = max
		self.err = 0.0
		self.value = 0.0
		self.i_part = 0.0
		self.dinput = 0.0
		self.__plot = plot

	def Update(self, dest, meas):
		prev_err = self.err
		self.err = dest-meas
		p_part = self.err * self.KP
		self.i_part += self.err*self.KI
		if self.i_part>self.max:
			self.i_part=self.max
		if self.i_part<self.min:
			self.i_part=self.min
			
		d_part = (self.err-prev_err)/self.dt*self.KD
		self.value = p_part + self.i_part + d_part
		
		if self.__plot:
			self.__plot.UpdateGraph(p_part,self.i_part,d_part)			

		if self.value>self.max:
			self.value=self.max
		if self.value<self.min:
			self.value=self.min
			
		return self.value
		
	def GetOut(self):
		return self.value
		
	def SetCoefs(self,kp,ki,kd):
		self.KP = kp
		self.KI = ki
		self.KD = kd	
		#~ self.co = 0.0
	
	def GetCoefs(self):
		return (self.KP,self.KI,self.KD)
		
	def GetError(self):
		return self.err		

class Empty(Framework):
	"""You can use this class as an outline for your tests.

	"""
	name = "Quadcopter" # Name of the to display
	description="The description text goes here"
	cut =1
	center = 0
	
	dp = 0.001
	di = 0.0001
	dd = 0.001
		
	def __init__(self):
		""" 
		Initialize all of your objects here.
		Be sure to call the Framework's initializer first.
		"""
		super(Empty, self).__init__()

		# Initialize all of the objects
		ground = self.world.CreateStaticBody(
			shapes=[ 
				b2EdgeShape(vertices=[(-40,0),(40,0)])
				#~ b2EdgeShape(vertices=[(-1.1,-40),(-1.1,40)]),
				]
		) 
		box=b2FixtureDef(
			shape=b2PolygonShape(box=(0.3,0.3)),
			density=0.01,
			friction=0.3)		
		
		self.q = [qcopter(self.world,self.renderer,1,0.05,-8,8,0,1)]##,qcopter(self.world,self.renderer,1,0.1,-12,8,0,0)]
	
		(self.kp,self.ki,self.kd)  = self.q[0].GetCoefs(self.q[0].configer)
		
		# The platform
		#~ fixture=b2FixtureDef(
			#~ shape=b2PolygonShape(box=(0.02,4.5)), 
			#~ density=1,
			#~ friction=0.6,
		#~ )			

		#~ self.platform=self.world.CreateDynamicBody(position=(-8,4.5), fixtures=fixture, )
		
		#~ self.platform.type=b2_staticBody
		if not(self.q[0].x_pid or self.q[0].y_pid):	
			self.world.CreateRevoluteJoint(
				bodyA=self.q[0].body,
				bodyB=ground,
				anchor=(self.q[0].GetPos().x,self.q[0].GetPos().y),
				maxMotorTorque=0,
				enableMotor=False
			)
				
		for i in xrange(10):
			for j in xrange(5+i/5):
				self.world.CreateDynamicBody(
					fixtures=box,
					position=(i*3-10, 1+1*j)
				)		
	
	def Keyboard(self, key):
		"""
		The key is from Keys.K_*
		(e.g., if key == Keys.K_z: ... )
		"""
		if key == Keys.K_u:
			self.kp+=self.dp
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		if key == Keys.K_j:
			self.kp-=self.dp
			if self.kp<0:
				self.kp=0.0
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		if key == Keys.K_i:
			self.ki+=self.di
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		if key == Keys.K_k:
			self.ki-=self.di
			if self.ki<0:
				self.ki=0.0
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		if key == Keys.K_o:
			self.kd+=self.dd
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		if key == Keys.K_l:
			self.kd-=self.dd
			if self.kd<0:
				self.kd=0.0
			self.q[0].SetCoefs(self.q[0].configer,self.kp,self.ki,self.kd)
		
		if key == Keys.K_f:
			self.center+=1
			if self.center>2:
				self.center = 0
		
		if key == Keys.K_t:
			self.cut=1-self.cut
			self.q[0].SetCut(self.cut)			
			
		if key == Keys.K_r:
			pass	
		
		if key == Keys.K_s:
			self.q[0].saveConf()

	def Step(self, settings):
		"""Called upon every step.
		You should always call
		 -> super(Your_Test_Class, self).Step(settings)
		at the beginning or end of your function.

		If placed at the beginning, it will cause the actual physics step to happen first.
		If placed at the end, it will cause the physics step to happen after your code.
		"""
		super(Empty, self).Step(settings)
		self.__period_divider = 0	
		for i in self.q:
			i.Update()
			if i.IsTargetComplete():
				i.NextTarget()
			i.DrawTargets()	
			i.DrawTarget()
				
		#~ self.renderer.DrawCircle(self.renderer.to_screen(self.q[0].GetTarget()),3,b2Color(0.25,0.5,0.99),1)
		self.Print("X: "+str(round(self.q[0].GetTarget().x,1))+" "+"Y: "+str(round(self.q[0].GetTarget().y,1)))
		self.Print("dist: "+str(round( sqrt(self.q[0].GetErr().x**2+self.q[0].GetErr().y**2),1)))
		#~ self.Print("co: "+str(round( -self.q[0].x_pid.co/3.14*180,1)))
		if self.q[0].x_pid:
			self.Print("co: "+str(round( -self.q[0].x_pid.value/3.14*180,1)))
		
		self.Print("vel: "+str(round(self.q[0].GetVel(),1)))
		self.Print("kp: "+str(self.kp)+" ki: "+str(self.ki)+" kd: "+str(self.kd))
		if self.center==1:
			self.viewCenter = ( (self.q[0].GetTarget().x+self.q[0].GetPos().x)/2, (self.q[0].GetTarget().y+self.q[0].GetPos().y)/2)
		elif self.center==2:
			self.viewCenter = self.q[0].GetPos()
		# do stuff
		# Placed after the physics step, it will draw on top of physics objects
	
	def ShiftMouseDown(self,p):
		for i,e in enumerate(self.q):
			#~ print p
			e.AddTarget(b2Vec2(p.x+8*i,p.y))
		#~ self.q.SetTarget(p)
				
	def ShapeDestroyed(self, shape):
		"""
		Callback indicating 'shape' has been destroyed.
		"""
		pass

	def JointDestroyed(self, joint):
		"""
		The joint passed in was removed.
		"""
		pass
		
	# More functions can be changed to allow for contact monitoring and such.
	# See the other testbed examples for more information.

if __name__=="__main__":
	main(Empty)

