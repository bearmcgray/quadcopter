from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import copy
from math import cos, sin

try:
    from PIL.Image import open
except ImportError, err:
    from Image import open

from ArcBall import * 				# ArcBallT and this tutorials set of points/vectors/matrix types

PI2 = 2.0*3.1415926535			# 2 * PI (not squared!) 		// PI Squared

# *********************** Globals *********************** 
# Python 2.2 defines these directly
try:
	True
except NameError:
	True = 1==1
	False = 1==0

dist = 6	

g_Transform = Matrix4fT ()
g_LastRot = Matrix3fT ()
g_ThisRot = Matrix3fT ()

g_ArcBall = ArcBallT (640, 480)
g_isDragging = False
g_quadratic = None
TopID = None
BotID = None
BackID = None
FrontID = None
LeftID = None
RightID = None

# A general OpenGL initialization function.  Sets all of the initial parameters. 
def Initialize (Width, Height):				# We call this right after our OpenGL window is created.
	global g_quadratic
	global TopID,BotID,BackID,FrontID,LeftID,RightID

	glClearColor(0.0, 0.5, 0.5, 0.0)					# This Will Clear The Background Color To Black
	glClearDepth(1.0)									# Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LEQUAL)								# The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST)								# Enables Depth Testing
	glShadeModel (GL_FLAT);								# Select Flat Shading (Nice Definition Of Objects)
	glHint (GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST) 	# Really Nice Perspective Calculations

	g_quadratic = gluNewQuadric();
	gluQuadricNormals(g_quadratic, GLU_SMOOTH);
	gluQuadricDrawStyle(g_quadratic, GLU_FILL); 
	# Why? this tutorial never maps any textures?! ? 
	# gluQuadricTexture(g_quadratic, GL_TRUE);			# // Create Texture Coords

	glEnable (GL_LIGHT0)
	glEnable (GL_LIGHTING)

	glEnable (GL_COLOR_MATERIAL)
	
	TopID = loadImage('TopInv.png')
	BotID = loadImage('BottomInv.png')
	
	FrontID = loadImage('FrontInv.png')
	BackID = loadImage('BackInv.png')

	LeftID = loadImage('LeftInv.png')
	RightID = loadImage('rightInv.png')

	return True




def Upon_Drag (cursor_x, cursor_y):
	""" Mouse cursor is moving
		Glut calls this function (when mouse button is down)
		and pases the mouse cursor postion in window coords as the mouse moves.
	"""
	global g_isDragging, g_LastRot, g_Transform, g_ThisRot

	if (g_isDragging):
		mouse_pt = Point2fT (cursor_x, cursor_y)
		ThisQuat = g_ArcBall.drag (mouse_pt)						# // Update End Vector And Get Rotation As Quaternion
		g_ThisRot = Matrix3fSetRotationFromQuat4f (ThisQuat)		# // Convert Quaternion Into Matrix3fT
		# Use correct Linear Algebra matrix multiplication C = A * B
		g_ThisRot = Matrix3fMulMatrix3f (g_LastRot, g_ThisRot)		# // Accumulate Last Rotation Into This One
		g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot)	# // Set Our Final Transform's Rotation From This One
	return

def Upon_Click (button, button_state, cursor_x, cursor_y):
	""" Mouse button clicked.
		Glut calls this function when a mouse button is
		clicked or released.
	"""
	global g_isDragging, g_LastRot, g_Transform, g_ThisRot,dist
	#~ print button, button_state, cursor_x, cursor_y
	g_isDragging = False
	if (button == GLUT_RIGHT_BUTTON and button_state == GLUT_UP):
		# Right button click
		g_LastRot = Matrix3fSetIdentity ();							# // Reset Rotation
		g_ThisRot = Matrix3fSetIdentity ();							# // Reset Rotation
		g_Transform = Matrix4fSetRotationFromMatrix3f (g_Transform, g_ThisRot);	# // Reset Rotation
	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_UP):
		# Left button released
		g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
	elif (button == GLUT_LEFT_BUTTON and button_state == GLUT_DOWN):
		# Left button clicked down
		g_LastRot = copy.copy (g_ThisRot);							# // Set Last Static Rotation To Last Dynamic One
		g_isDragging = True											# // Prepare For Dragging
		mouse_pt = Point2fT (cursor_x, cursor_y)
		g_ArcBall.click (mouse_pt);								# // Update Start Vector And Prepare For Dragging
	elif (button ==3 ):
		dist -=0.1
	elif (button ==4 ):
		dist +=0.1
	return



def Torus(MinorRadius, MajorRadius):		
	# // Draw A Torus With Normals
	glBegin( GL_TRIANGLE_STRIP );									# // Start A Triangle Strip
	for i in xrange (20): 											# // Stacks
		for j in xrange (-1, 20): 										# // Slices
			# NOTE, python's definition of modulus for negative numbers returns
			# results different than C's
			#       (a / d)*d  +  a % d = a
			if (j < 0):
				wrapFrac = (-j%20)/20.0
				wrapFrac *= -1.0
			else:
				wrapFrac = (j%20)/20.0;
			phi = PI2*wrapFrac;
			sinphi = sin(phi);
			cosphi = cos(phi);

			r = MajorRadius + MinorRadius*cosphi;

			glNormal3f (sin(PI2*(i%20+wrapFrac)/20.0)*cosphi, sinphi, cos(PI2*(i%20+wrapFrac)/20.0)*cosphi);
			glVertex3f (sin(PI2*(i%20+wrapFrac)/20.0)*r, MinorRadius*sinphi, cos(PI2*(i%20+wrapFrac)/20.0)*r);

			glNormal3f (sin(PI2*(i+1%20+wrapFrac)/20.0)*cosphi, sinphi, cos(PI2*(i+1%20+wrapFrac)/20.0)*cosphi);
			glVertex3f (sin(PI2*(i+1%20+wrapFrac)/20.0)*r, MinorRadius*sinphi, cos(PI2*(i+1%20+wrapFrac)/20.0)*r);
	glEnd();														# // Done Torus
	return

def loadImage( imageName ):
	"""Load an image file as a 2D texture using PIL"""
	im = open(imageName)
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


def Cube(id):
	ax=146./100.
	ay=102./100.    
	az=51./100.
	setupTexture(id[0])
	glBegin(GL_QUADS)
	glTexCoord2f(0.0, 0.0); glVertex3f( -ax/2, -ay/2, az/2);
	glTexCoord2f(1.0, 0.0); glVertex3f(  ax/2,  -ay/2, az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f(  ax/2,   ay/2, az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( -ax/2,  ay/2, az/2);
	glEnd()

	setupTexture(id[1])
	glBegin(GL_QUADS)
	glTexCoord2f(0.0, 0.0); glVertex3f( -ax/2,  ay/2, -az/2);
	glTexCoord2f(1.0, 0.0); glVertex3f(  ax/2,  ay/2, -az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f(  ax/2, -ay/2, -az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( -ax/2, -ay/2, -az/2);
	glEnd()

	setupTexture(id[2])
	glBegin(GL_QUADS)
	glTexCoord2f(1.0, 0.0); glVertex3f(-ax/2,  ay/2, -az/2);
	glTexCoord2f(0.0, 0.0); glVertex3f( ax/2,  ay/2, -az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( ax/2,  ay/2,  az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f( -ax/2,  ay/2,  az/2);
	glEnd()
	
	setupTexture(id[3])
	glBegin(GL_QUADS)
	glTexCoord2f(0.0, 0.0); glVertex3f( -ax/2,  -ay/2, -az/2);
	glTexCoord2f(1.0, 0.0); glVertex3f(  ax/2,  -ay/2, -az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f(  ax/2,  -ay/2,  az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( -ax/2,  -ay/2,  az/2);
	glEnd()

	setupTexture(id[4])
	glBegin(GL_QUADS)
	glTexCoord2f(1.0, 0.0); glVertex3f( -ax/2,  -ay/2, -az/2);
	glTexCoord2f(0.0, 0.0); glVertex3f( -ax/2,   ay/2, -az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( -ax/2,   ay/2,  az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f( -ax/2,  -ay/2,  az/2);
	glEnd()
	
	setupTexture(id[5])
	glBegin(GL_QUADS)
	glTexCoord2f(0.0, 0.0); glVertex3f( ax/2,  -ay/2, -az/2);
	glTexCoord2f(1.0, 0.0); glVertex3f( ax/2,   ay/2, -az/2);
	glTexCoord2f(1.0, 1.0); glVertex3f( ax/2,   ay/2,  az/2);
	glTexCoord2f(0.0, 1.0); glVertex3f( ax/2,  -ay/2,  az/2);
	glEnd()

def Draw ():
	global TopID,BotID,BackID,FrontID,LeftID,RightID,dist
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				# // Clear Screen And Depth Buffer
	glLoadIdentity();												# // Reset The Current Modelview Matrix
	#~ glTranslatef(-1.5,0.0,-6.0);									# // Move Left 1.5 Units And Into The Screen 6.0
	glTranslatef(0,0.0,-dist);									# // Move Left 1.5 Units And Into The Screen 6.0

	glPushMatrix();													# // NEW: Prepare Dynamic Transform
	glMultMatrixf(g_Transform);										# // NEW: Apply Dynamic Transform
	glColor3f(0.75,0.75,1.0);
	#~ Torus(0.30,1.00);
	
	#~ print "qq"
	Cube([TopID,BotID,BackID,FrontID,LeftID,RightID])
	glPopMatrix();													# // NEW: Unapply Dynamic Transform

#~	glLoadIdentity();												# // Reset The Current Modelview Matrix
	#~ glTranslatef(1.5,0.0,-6.0);										# // Move Right 1.5 Units And Into The Screen 7.0

	#~ glPushMatrix();													# // NEW: Prepare Dynamic Transform
	#~ glMultMatrixf(g_Transform);										# // NEW: Apply Dynamic Transform
	#~ glColor3f(1.0,0.75,0.75);
	#~ gluSphere(g_quadratic,1.3,20,20);
	#~ glPopMatrix();													# // NEW: Unapply Dynamic Transform

	glFlush ();														# // Flush The GL Rendering Pipeline
	glutSwapBuffers()
	
	return