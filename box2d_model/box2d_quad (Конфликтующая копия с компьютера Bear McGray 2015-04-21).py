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
from math import sin,cos,sqrt
#~ from random import random

class filter:
	def __init__(self,a):
		self.__a = float(a)
		self.__p = 0.0
		
	def __call__(self,e):
		self.__p = self.__p*(1-self.__a) + e*self.__a 
		return self.__p

class graph:
	def __init__(self,r,k,x0,y0,w,h):
		self.__x0 = x0
		self.__k = k
		self.__y0 = y0
		self.__w = w
		self.__h = h
		self.__r = r
		self.__data=[(0,0,0)]*self.__w
		#~ print self.__data
		self.__c1 = int(self.__y0+self.__h/3.)
		self.__c2 = int(self.__y0+self.__h*2./3.)
		self.__c3 = int(self.__y0+self.__h)
		
	def UpdateGraph(self,p1,p2,p3):
		#~ del self.__data[0]
		self.__data=self.__data[1:]
		self.__data.append( (self.__c1-int(self.__k*p1),self.__c2-int(self.__k*p2),self.__c3-int(self.__k*p3)) )
		
		#~ v = [(self.__x0,self.__y0),(self.__x0+self.__w,self.__y0),(self.__x0+self.__w,self.__y0+self.__h),(self.__x0,self.__y0+self.__h)]
		#~ self.__r.DrawPolygon(v, b2Color(1,1,1))
		for i in xrange(len(self.__data)):
			self.__r.DrawPoint((self.__x0+i,self.__data[i][0]),0,b2Color(0.5,0.25,0.75))
			self.__r.DrawPoint((self.__x0+i,self.__data[i][1]),0,b2Color(0.75,0.25,0.5))
			self.__r.DrawPoint((self.__x0+i,self.__data[i][2]),0,b2Color(0.5,0.75,0.25))
		

class qcopter:
	body = None
	THRUST_KOEF = 5
	cut = 1
	thrust = 0.0

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
		if mode:
			#~ self.a_pid = PID_typeC(2.0, 0.1, 6.0,  0.5, -0.5)
			#~ self.a_pid = PID_typeC(1.3, 0.03, 1.0,  0.15, -0.15)
			T = 1/1.
			#~ self.a_pid = PID_typeC(0.9, 0.0082/T, 1.1*T, T, 0.15, -0.15,None)
			self.a_pid = PID_typeC(0.13, 0.00175, 0.88, 1, 0.05, -0.05,None)##graph(self.renderer,30,300,10,100,100))
			
			#~ self.y_pid = PID_typeC(6.2, 0.2, 17.4, 0.5, 0)
			
			#~ self.y_pid = PID_typeC(1.0, 0.01/T, 1.9*T, T, 0.5, 0.08)
			self.y_pid = PID_typeC(1.5, 0.015/T, 2.5*T, T, 0.25, -0.01,None)
			#~ self.y_pid = PID_typeC(0.0, 0.00/T, 0.0*T, T, 0.5, 0.08)
			
			self.x_pid = PID_typeC(1.83, 0.00141, 1.62, 1, 3.14/3, -3.14/3,graph(self.renderer,30,300,10,100,100))
			#~ self.x_pid = PID_typeC(1.4, 0.01, 2.4, 3.14/6, -3.14/6)
			#~ self.x_pid = PID_typeC(0.0, 0.0, 0.0, 3.14/6, -3.14/6)
			#~ self.x_pid = PID_typeC(0.260, 0.001/T, 0.072*T, T, 3.14/6, -3.14/6,graph(self.renderer,30,300,10,100,100))
			self.thrust_color = b2Color(1,0,0.5)
		else:
			self.a_pid = PIDz(2.0, 0.1, 6.0,  0.5, -0.5)
			self.y_pid = PIDz(6.2, 0.2, 17.4, 0.5, 0)
			self.x_pid = PIDz(0.6, 0.02, 22, 3.14/8, -3.14/8)
			self.thrust_color = b2Color(0,0,1)
		
		rail=b2FixtureDef(
			shape=b2PolygonShape(box=(self.a,self.b)),
			density=0.09,
			friction=0.01)
		body=b2FixtureDef(
			shape=b2CircleShape(radius=a/2.),
			density=0.00,
			friction=0.01)
			
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
		self.renderer.DrawCircle(self.renderer.to_screen(b2Vec2(self.x_tgt,self.y_tgt)),3,b2Color(0.5,0.99,0.25),1)
		
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

		if self.body.angle>3.1415:
			self.body.angle -= 3.1415*2
		if self.body.angle<-3.1415:
			self.body.angle += 3.1415*2
			
		alpha_rnd =0* b2Random(-3.14/180.0/2, 3.14/180.0/2)
		h_rnd = 0*b2Random(-0.002,0.002)
		
		cur_angle_with_noise = self.__f(self.body.angle + alpha_rnd)
		#~ print self.x_tgt,self.body.position.x
		pid_ang   = self.x_pid.Update(self.x_tgt, self.body.position.x)
		#~ pid_ang   = 0
		
		d_torque =  self.a_pid.Update(-pid_ang, cur_angle_with_noise)
		y_torque =   self.y_pid.Update(self.y_tgt, self.body.position.y+h_rnd)		
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
		transform1.Set(plt,self.body.angle+3.14/2)
		transform2.Set(prt,self.body.angle+3.14/2)
		polygon1 = b2PolygonShape(box=(tl*self.THRUST_KOEF,0.01))
		polygon2 = b2PolygonShape(box=(tr*self.THRUST_KOEF,0.01))
		#~ self.thrust_color = b2Color(1,0.5,0.25)
		for shape, transform in [(polygon1, transform1),(polygon2, transform2)]:
			new_verts = [self.renderer.to_screen(transform*v) for v in shape.vertices]
			self.renderer.DrawSolidPolygon(new_verts, self.thrust_color)
		
		#~ self.renderer.DrawCircle(self.renderer.to_screen(b2Vec2(self.x_tgt,self.y_tgt)),3,b2Color(0.25,0.5,0.99),1)
		#~ self.viewCenter = ((self.body.position.x+self.x_tgt)/2, (self.body.position.y+self.y_tgt)/2)
		
	def GetErr(self):
		return b2Vec2(self.x_tgt-self.body.position.x, self.y_tgt-self.body.position.y)
		
	def GetTarget(self):
		return b2Vec2(self.x_tgt,self.y_tgt)		
	
	def SetCoefs(self,kp,ki,kd):
		self.x_pid.SetCoefs(kp,ki,kd)
	
	def GetCoefs(self):
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
		
	def SetCoefs(self,kp,ki,kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd	
		self.co = 0.0
	
	def GetCoefs(self):
		return (self.kp,self.ki,self.kd)
	
	def Update(self,sp,pv):
		e = float(sp) - float(pv)
		del self.pv[0]
		self.pv.append(pv)
		k = -1## dont look at me
		p_part = -self.kp*(self.pv[k]-self.pv[k-1])
		i_part  = self.ki*e
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
			
		return self.co		
		
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
	def __init__(self,kp,ki,kd,dt,max,min):
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

	def Update(self, dest, meas):
		prev_err = self.err
		self.err = dest-meas
		#~ print self.err
		p_part = self.err * self.KP
		self.i_part += self.err*self.KI
		if self.i_part>self.max:
			self.i_part=self.max
			#~ print "int max"
		if self.i_part<self.min:
			self.i_part=self.min
			#~ print "int min"
			
		d_part = -(self.err-prev_err)/self.dt*self.KD	
		d_part = (self.err-prev_err)/self.dt*self.KD
		#~ self.value = p_part 
		self.value = p_part + self.i_part + d_part

		if self.value>self.max:
			self.value=self.max
		if self.value<self.min:
			self.value=self.min
			
		#~ print self.value
		return self.value
		
	def GetOut(self):
		return self.value
	
		

class Empty(Framework):
	"""You can use this class as an outline for your tests.

	"""
	name = "Quadcopter" # Name of the class to display
	description="The description text goes here"
	cut =1
	center = 0
	
	dp = 0.01
	di = 0.00001
	dd = 0.01
	
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
			density=0.1,
			friction=0.3)		
		
		self.q = [qcopter(self.world,self.renderer,1,0.05,-8,8,0,1)]##,qcopter(self.world,self.renderer,1,0.1,-12,8,0,0)]
				
		(self.kp,self.ki,self.kd)  = self.q[0].GetCoefs()
		
		# The platform
		fixture=b2FixtureDef(
			shape=b2PolygonShape(box=(0.02,4.5)), 
			density=2,
			friction=0.6,
		)			

		#~ self.platform=self.world.CreateDynamicBody(position=(-8,4.5), fixtures=fixture, )
		
		#~ self.platform.type=b2_staticBody
			
		#~ self.world.CreateRevoluteJoint(
			#~ bodyA=self.q[0].body,
			#~ bodyB=self.platform,
			#~ anchor=(-8,8),
			#~ maxMotorTorque=0,
			#~ enableMotor=False
		#~ )
				
		##		
				
		#~ self.q.SetCoefs(self.kp,self.ki,self.kd)
		
		#~ for i in xrange(20):
			#~ for j in xrange(10+i/5):
				#~ self.world.CreateDynamicBody(
					#~ fixtures=box,
					#~ position=(i*3-10, 1+1*j)
				#~ )		
	

	def Keyboard(self, key):
		"""
		The key is from Keys.K_*
		(e.g., if key == Keys.K_z: ... )
		"""
		if key == Keys.K_u:
			self.kp+=self.dp
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		if key == Keys.K_j:
			self.kp-=self.dp
			if self.kp<0:
				self.kp=0.0
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		if key == Keys.K_i:
			self.ki+=self.di
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		if key == Keys.K_k:
			self.ki-=self.di
			if self.ki<0:
				self.ki=0.0
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		if key == Keys.K_o:
			self.kd+=self.dd
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		if key == Keys.K_l:
			self.kd-=self.dd
			if self.kd<0:
				self.kd=0.0
			self.q[0].SetCoefs(self.kp,self.ki,self.kd)
		
		if key == Keys.K_g:
			self.center+=1
			if self.center>2:
				self.center = 0
		
		if key == Keys.K_t:
			self.cut=1-self.cut
			self.q[0].SetCut(self.cut)			

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
		self.Print("vel: "+str(round(self.q[0].GetVel(),1)))
		self.Print("kp: "+str(self.kp)+" ki: "+str(self.ki)+" kd: "+str(self.kd))
		if self.center==1:
			self.viewCenter = ( (self.q[0].GetTarget().x+self.q[0].GetPos().x)/2, (self.q[0].GetTarget().y+self.q[0].GetPos().y)/2)
		elif self.center==2:
			self.viewCenter = self.q[0].GetPos()
		# do stuff
		# Placed after the physics step, it will draw on top of physics objects
	
	def ShiftMouseDown(self,p):
		for i in self.q:
			i.AddTarget(p)
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

