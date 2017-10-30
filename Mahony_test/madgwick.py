from math import atan2,asin

def invSqrt(x):
	return 1/(x)**(0.5)

class Madgwick:
	#~ __beta = 0.5
	def __init__(self,dt,beta):
		self.__dt = 1.0/dt
		self.__beta = beta

		self.__ax = 0
		self.__ay = 0
		self.__az = 0

		self.__q0 = 1.0##1/(2**(0.5))##1.0
		self.__q1 = 0.0##1/(2**(0.5))##0.0
		self.__q2 = 0.0
		self.__q3 = 0.0	## quaternion of sensor frame relative to auxiliary frame


	def MadgwickAHRSupdate(self, gx, gy, gz, ax, ay, az, mx, my, mz):

		## Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if (mx == 0.0) and (my == 0.0) and (mz == 0.0):
			#~ raise RuntimeError("bad magneto")
			self.MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az)
			#~ print "*"*42
			return

		## Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-self.__q1 * gx - self.__q2 * gy - self.__q3 * gz)
		qDot2 = 0.5 * (self.__q0 * gx + self.__q2 * gz - self.__q3 * gy)
		qDot3 = 0.5 * (self.__q0 * gy - self.__q1 * gz + self.__q3 * gx)
		qDot4 = 0.5 * (self.__q0 * gz + self.__q1 * gy - self.__q2 * gx)

		## Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not( (ax == 0.0) and (ay == 0.0) and (az == 0.0)):

			## Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az)
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm
			self.__ax = ax
			self.__ay = ay
			self.__az = az
			#~ print ax,ay,az

			## Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz)
			mx *= recipNorm
			my *= recipNorm
			mz *= recipNorm

			#~ print mx,my,mz

			## Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * self.__q0 * mx
			_2q0my = 2.0 * self.__q0 * my
			_2q0mz = 2.0 * self.__q0 * mz
			_2q1mx = 2.0 * self.__q1 * mx
			_2q0 = 2.0 * self.__q0
			_2q1 = 2.0 * self.__q1
			_2q2 = 2.0 * self.__q2
			_2q3 = 2.0 * self.__q3
			_2q0q2 = 2.0 * self.__q0 * self.__q2
			_2q2q3 = 2.0 * self.__q2 * self.__q3
			q0q0 = self.__q0 * self.__q0
			q0q1 = self.__q0 * self.__q1
			q0q2 = self.__q0 * self.__q2
			q0q3 = self.__q0 * self.__q3
			q1q1 = self.__q1 * self.__q1
			q1q2 = self.__q1 * self.__q2
			q1q3 = self.__q1 * self.__q3
			q2q2 = self.__q2 * self.__q2
			q2q3 = self.__q2 * self.__q3
			q3q3 = self.__q3 * self.__q3

			## Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * self.__q3 + _2q0mz * self.__q2 + mx * q1q1 + _2q1 * my * self.__q2 + _2q1 * mz * self.__q3 - mx * q2q2 - mx * q3q3
			hy = _2q0mx * self.__q3 + my * q0q0 - _2q0mz * self.__q1 + _2q1mx * self.__q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.__q3 - my * q3q3
			_2bx = (hx * hx + hy * hy)**(0.5)
			_2bz = -_2q0mx * self.__q2 + _2q0my * self.__q1 + mz * q0q0 + _2q1mx * self.__q3 - mz * q1q1 + _2q2 * my * self.__q3 - mz * q2q2 + mz * q3q3
			_4bx = 2.0 * _2bx
			_4bz = 2.0 * _2bz
			#~ _8bz = 2.0 * _4bz
			#~ _8bx = 2.0 * _4bx

			## Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.__q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.__q3 + _2bz * self.__q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.__q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.__q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.__q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.__q2 + _2bz * self.__q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.__q3 - _4bz * self.__q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.__q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.__q2 - _2bz * self.__q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.__q1 + _2bz * self.__q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.__q0 - _4bz * self.__q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.__q3 + _2bz * self.__q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.__q0 + _2bz * self.__q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.__q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			#~ s0= -_2q2*(2*(q1q3 - q0q2) - ax)    +   _2q1*(2*(q0q1 + q2q3) - ay)   +  -_4bz*self.__q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*self.__q3+_4bz*self.__q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*self.__q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz)
			#~ s1= _2q3*(2*(q1q3 - q0q2) - ax) +   _2q0*(2*(q0q1 + q2q3) - ay) +   -4*self.__q1*(2*(0.5 - q1q1 - q2q2) - az)    +   _4bz*self.__q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*self.__q2+_4bz*self.__q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*self.__q3-_8bz*self.__q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz)
			#~ s2= -_2q0*(2*(q1q3 - q0q2) - ax)    +     _2q3*(2*(q0q1 + q2q3) - ay)   +   (-4*self.__q2)*(2*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*self.__q2-_4bz*self.__q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*self.__q1+_4bz*self.__q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*self.__q0-_8bz*self.__q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz)
			#~ s3= _2q1*(2*(q1q3 - q0q2) - ax) +   _2q2*(2*(q0q1 + q2q3) - ay)+(-_8bx*self.__q3+_4bz*self.__q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*self.__q0+_4bz*self.__q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*self.__q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz)
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) ## normalise step magnitude
			s0 *= recipNorm
			s1 *= recipNorm
			s2 *= recipNorm
			s3 *= recipNorm

			## Apply feedback step
			qDot1 -= self.__beta * s0
			qDot2 -= self.__beta * s1
			qDot3 -= self.__beta * s2
			qDot4 -= self.__beta * s3

		## Integrate rate of change of quaternion to yield quaternion
		self.__q0 += qDot1 * (1.0 / self.__dt)
		self.__q1 += qDot2 * (1.0 / self.__dt)
		self.__q2 += qDot3 * (1.0 / self.__dt)
		self.__q3 += qDot4 * (1.0 / self.__dt)

		## Normalise quaternion
		recipNorm = invSqrt(self.__q0 * self.__q0 + self.__q1 * self.__q1 + self.__q2 * self.__q2 + self.__q3 * self.__q3)
		self.__q0 *= recipNorm
		self.__q1 *= recipNorm
		self.__q2 *= recipNorm
		self.__q3 *= recipNorm

	def MadgwickAHRSupdateIMU(self, gx, gy, gz, ax, ay, az):

		## Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-self.__q1 * gx - self.__q2 * gy - self.__q3 * gz)
		qDot2 = 0.5 * (self.__q0 * gx + self.__q2 * gz - self.__q3 * gy)
		qDot3 = 0.5 * (self.__q0 * gy - self.__q1 * gz + self.__q3 * gx)
		qDot4 = 0.5 * (self.__q0 * gz + self.__q1 * gy - self.__q2 * gx)

		## Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not( (ax == 0.0) and (ay == 0.0) and (az == 0.0)):

			## Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az)
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm

			## Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * self.__q0
			_2q1 = 2.0 * self.__q1
			_2q2 = 2.0 * self.__q2
			_2q3 = 2.0 * self.__q3
			_4q0 = 4.0 * self.__q0
			_4q1 = 4.0 * self.__q1
			_4q2 = 4.0 * self.__q2
			_8q1 = 8.0 * self.__q1
			_8q2 = 8.0 * self.__q2
			q0q0 = self.__q0 * self.__q0
			q1q1 = self.__q1 * self.__q1
			q2q2 = self.__q2 * self.__q2
			q3q3 = self.__q3 * self.__q3

			## Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.__q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
			s2 = 4.0 * q0q0 * self.__q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
			s3 = 4.0 * q1q1 * self.__q3 - _2q1 * ax + 4.0 * q2q2 * self.__q3 - _2q2 * ay

			## normalise step magnitude
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
			s0 *= recipNorm
			s1 *= recipNorm
			s2 *= recipNorm
			s3 *= recipNorm

			## Apply feedback step
			qDot1 -= self.__beta * s0
			qDot2 -= self.__beta * s1
			qDot3 -= self.__beta * s2
			qDot4 -= self.__beta * s3

		## Integrate rate of change of quaternion to yield quaternion
		self.__q0 += qDot1 * (1.0 / self.__dt)
		self.__q1 += qDot2 * (1.0 / self.__dt)
		self.__q2 += qDot3 * (1.0 / self.__dt)
		self.__q3 += qDot4 * (1.0 / self.__dt)

		## Normalise quaternion
		recipNorm = invSqrt(self.__q0 * self.__q0 + self.__q1 * self.__q1 + self.__q2 * self.__q2 + self.__q3 * self.__q3)
		self.__q0 *= recipNorm
		self.__q1 *= recipNorm
		self.__q2 *= recipNorm
		self.__q3 *= recipNorm

	def Set(self,q0,q1,q2,q3):
		self.__q0 = q0
		self.__q1 = q1
		self.__q2 = q2
		self.__q3 = q3
		
	def Get(self):
		return (self.__q0,self.__q1,self.__q2,self.__q3)

	def GetNormAccel(self):
		return (self.__ax,self.__ay,self.__az)

	def GetEuler(self):
		"""
			body 3-2-1 tait-bryan angles yaw pitch roll
			phi	- roll about x axis
			theta - pitch about y axis
			psi	- yaw about z axis
		"""
		roll = atan2(2*(self.__q0*self.__q1+self.__q2*self.__q3), 1-2*(self.__q1**2+self.__q2**2))
		pitch =asin(2*(self.__q0*self.__q2-self.__q1*self.__q3))
		yaw = atan2(2*(self.__q0*self.__q3+self.__q1*self.__q2),1-2*(self.__q2**2+self.__q3**2))
		return {"yaw":yaw,"pitch":pitch,"roll":roll}


if __name__=="__main__":
	m = Madgwick(1/50.,0.1)

	while 1:
		m.MadgwickAHRSupdate(0, 0, 0, 0.0, 100., 0.0, 2., 0.2, 0.2)
		print m.Get()
