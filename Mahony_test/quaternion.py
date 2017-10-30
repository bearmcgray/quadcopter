from math import cos

class Quaternion:
	def __init__(self,w,p,q,r):
		self.__w = float(w)
		self.__p = float(p)
		self.__q = float(q)
		self.__r = float(r)
	
	def __mul__(self,other):
		return Quaternion(
		self.__w*other.__w - self.__p*other.__p - self.__q*other.__q - self.__r*other.__r,
		
		self.__w*other.__p + self.__p*other.__w + self.__q*other.__r - self.__r*other.__q,
		self.__w*other.__q - self.__p*other.__r + self.__q*other.__w + self.__r*other.__p,
		self.__w*other.__r + self.__p*other.__q - self.__q*other.__p + self.__r*other.__w,
		)
		
	def __add__(self,other):	
		return Quaternion(other.__w+self.__w, other.__p+self.__p, other.__q+self.__q, other.__r+self.__r)

	def __sub__(self,other):	
		return Quaternion(other.__w-self.__w, other.__p-self.__p, other.__q-self.__q, other.__r-self.__r)
	
	def Conj(self):
		return Quaternion(self.__w, -self.__p, -self.__q, -self.__r)
		
	def __str__(self):
		return str([self.__w,self.__p,self.__q,self.__r])
	
	def __repr__(self):
		return self.__str__()
	
	def Mod(self):
		return (self.__w*self.__w+self.__p*self.__p+self.__q*self.__q+self.__r*self.__r)**(0.5)
		
	def Norm(self):
		norm = self.Mod()
		return Quaternion(self.__w/norm,	self.__p/norm, self.__q/norm, self.__r/norm)
	
	def At(self,p):
		if p==0:
			return self.__w
		elif p==1:
			return self.__p
		elif p==2:
			return self.__q
		elif p==3:
			return self.__r
		else:
			raise RuntimeError("bad quaternion index")

		
if __name__ == "__main__":
	q = Quaternion(cos(30/57.3/2),1.,0.,0.)
	print q*q*(q.Conj())
	