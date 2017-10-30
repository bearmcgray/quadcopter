#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#~ __NAME = '..\opengl\samples.txt'
#~ __NAME = '..\\opengl\\test.txt'
__NAME = 'haired2.dat'
__DIVK = 1.

def main():
	app = QtGui.QApplication([])
	file = open(__NAME,"r")

	legends=['gx','gy','gz','ax','ay','az','yaw','pitch','roll']
	colors = [(255,0,0),(0,255,0),(0,0,255),(255,0,255),(255,255,0),(0,255,255),(200,200,200),(50,100,150),(200,10,50)]
	y=[[],[],[],[],[],[],[],[],[]]
	points = 0
	min = 0
	max = 0
	rline = 0
	while 1:
		if rline ==0:
			rline =1
			file.readline()
			continue

		rl = file.readline()[:-1]
		if not rl:
			break
		srl = rl.split(' ')

		for i in xrange(len(y)):
			tmp = float(srl[i])
			if tmp>max:
				max = tmp
			if tmp<min:
				min = tmp

		y[0].append(float(srl[0]))
		y[1].append(float(srl[1]))
		y[2].append(float(srl[2]))
		y[3].append(float(srl[3])/__DIVK)
		y[4].append(float(srl[4])/__DIVK)
		y[5].append(float(srl[5])/__DIVK)
		y[6].append(float(srl[6]))
		y[7].append(float(srl[7]))
		y[8].append(float(srl[8]))

		points+=1

	file.close()

	p = pg.plot()
	p.setTitle('loaded'+__NAME)
	p.setRange(QtCore.QRectF(0, min-1, points, max-min+1))
	p.setLabel('bottom', 'Index', units='samples')
	p.addLegend()
	plt_grid = pg.GridItem()
	p.addItem(plt_grid)
	curve=[]
	for i in xrange(len(y)):
		curve.append(p.plot( pen=colors[i], name=legends[i]))
		curve[i].setData(y[i])

	## run qt
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
	main()

