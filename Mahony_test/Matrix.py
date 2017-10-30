
class Matrix:
	def __init__(self,m):
		if isinstance(m,str):
			r = m.split(";")
			self.__row = len(r)
			self.__m = []
			#~ print r
			for i in r:
				c = i.split()
				self.__col = len(c)
				tmp = []
				for j in c:
					tmp.append(float(j))
				self.__m.append(tmp)	
		elif isinstance(m,list):
			self.__m = m
			self.__row = len(m)
			self.__col = len(m[0])
		else:
			raise RuntimeError("bad matrix")
		#~ print self.__m
	
	def __make(self,r,c):
		tmp_m = []
		for i in xrange(r):
			tmp = []
			for j in xrange(c):
				tmp.append(0.0)
			tmp_m.append(tmp)
		return tmp_m
		
	def Det(self):
		n = self.__isSqure()
		if n==1:
			return self.At(0,0)
		#~ elif n==2:
			#~ return self.At(0,0)*self.At(1,1)-self.At(0,1)*self.At(1,0)
		else:
			tmp_d=0
			for i in xrange(n):
				tmp_d+=((-1)**(i+1))*self.At(i,1)*(self.Minor(i,1).Det())
			return tmp_d 	
	
	def Minor(self,r,c):
		(mr,mc) = self.GetDim()
		tmp_m = self.__make(mr,mc)
		for i in xrange(mr):
			for j in xrange(mc):
				tmp_m[i][j] = self.At(i,j)
		del tmp_m[r]
		for i in tmp_m:
			del i[c]
		return Matrix(tmp_m)
	
	def __isSqure(self):
		(n,n2)=self.GetDim()
		if n!=n2:
			raise RuntimeError("Matrix need to be square for this type of operation")
		return n	
		
	def Adj(self):
		n = self.__isSqure()
		tmp_m = self.__make(n,n)
		for i in xrange(n):
			for j in xrange(n):
				tmp_m[i][j] = ((-1)**(i+j))*self.Minor(i,j).Det()
		return Matrix(tmp_m)
	
	def Inverse(self):
		det = self.Det()
		if det == 0 or not self.__isSqure():
			raise RuntimeError("Matrix non invertable in case of zero determinant")
		return self.Transpose().Adj().ConstMul(1/det)
	
	def Transpose(self):
		(r,c)=self.GetDim()
		tmp_m = self.__make(c,r)
		for i in xrange(r):
			for j in xrange(c):
				tmp_m[j][i] = self.At(i,j)
		return Matrix(tmp_m)
		
	def GetDim(self):
		return (self.__row,self.__col)
	
	def __str__(self):
		return str(self.__m)

	def __repr__(self):
		return str(self.__m)
		
	def DotMul(self,m):
		(mr,mc)=m.GetDim()
		if self.__col!=mr:
			raise RuntimeError("matrix dimmensions does not match: "+str(self.__row)+" x "+str(self.__col)+" & "+str(mr)+" x "+str(mc))
			
		tmp_m = self.__make(self.__row,mc)	
		for i in xrange(self.__row):
			for j in xrange(mc):
				for k in xrange(mr):
					tmp_m[i][j] += self.At(i,k)*m.At(k,j)
					
		return Matrix(tmp_m)
	
	def ConstMul(self,const):
		tmp_m = []
		for r in self.__m:
			tmp = []
			for c in r:
				tmp.append(c*const)
			tmp_m.append(tmp)
		return Matrix(tmp_m)

	def Add(self,m):
		tmp_m = []
		for ri in xrange(self.__row):
			tmp = []
			for ci in xrange(self.__col):
				tmp.append(self.At(ri,ci)+m.At(ri,ci))
			tmp_m.append(tmp)
		return Matrix(tmp_m)

	def Sub(self,m):
		tmp_m = []
		for ri in xrange(self.__row):
			tmp = []
			for ci in xrange(self.__col):
				tmp.append(self.At(ri,ci)-m.At(ri,ci))
			tmp_m.append(tmp)
		return Matrix(tmp_m)
	
	def At(self,r,c):
		return self.__m[r][c]
	
	@staticmethod
	def I(n):
		tmp_m = Matrix("").__make(n,n)
		for i in xrange(n):
			for j in xrange(n):
				tmp_m[i][j] = 1 if i==j else 0
			
		return Matrix(tmp_m)		
	
	@staticmethod
	def Zero(r,c):
		tmp_m = Matrix("").__make(r,c)
		return Matrix(tmp_m)	
	
if __name__ == "__main__":
		print Matrix("2 2 3; 4 5 6; 7 8 9").Inverse()
		print Matrix("1 2; 3 4").Inverse()
		print Matrix.I(4)
		print Matrix.Zero(3,2)
		