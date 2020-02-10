import numpy as np 

numobjects=4		# these values are normally only knewn by the dimension of the inputs
numpoints=500

# create random SLAM output
points = np.random.random((3,numpoints))
points = points*20
n_p=points.size/3					# calcultes the number of points in the map

# create usual image size
pixels=np.matrix([320,240]) 				# image size

# create random tensor flow output
classes = np.arange(1,numobjects+1)
confidences = np.random.random((numobjects,1))
bb = np.random.random((numobjects,4))
bb = bb*0.5
bb[:,1]=bb[:,0]*2*pixels[0,0]
bb[:,3]=bb[:,2]*2*pixels[0,1]
bb[:,0]=bb[:,0]*pixels[0,0]
bb[:,2]=bb[:,2]*pixels[0,1]
n_o=bb.size/4


# create random camera position
T=np.matrix([0.,0.,0.])					# Position of Camera global
T=np.transpose(T)
R=np.matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])		# Rotation Matrix of Camera
f=35.0 							# focal length



##==============================================================================================##
##				Actual skript starts here					##
##==============================================================================================##

# Main Program is below


#--coordtransform--#
# transforms the X vector of the used point [i] into pixel coordinates
# the X vector in pixel Coordinates Xu is returned
def coordtransform(i):
	X=points[:,i]
	XT = (R.T)*(X-T)
	Xu = -np.matrix([[f*XT.item(0)/XT.item(2)],[f*XT.item(1)/XT.item(2)]])+c
	return Xu


#--labelling--#
# checks whether the point [i] is in the bounding box corresponding to object [j]
# one or zero are returned for true or false
def labelling(j,i):
	if Xu.item(0)>bb[j,0] and Xu.item(0)<bb[j,1] and Xu.item(1)>bb[j,2] and Xu.item(1)<bb[j,3]:
		label=1
	else:
		label=0
	return label


#--text_label--#
# checks whether this point is closer to the center of the bounding box than the prevously closest
# returns the point and the new distance if it is closer
# returns the old disttance and a -1 for wrong when it is not closer
def text_label(j,i,distalt):
	center=np.matrix([bb[j,1]-bb[j,0],bb[j,3]-bb[j,2]])
	dist=np.linalg.norm(Xu-center)
	if distalt==999.0:
		lp=i
		distalt=dist
	else:
		if dist<distalt:
			lp=i
			distalt=dist
		else:
			lp=-1
	return [lp,distalt]


# Image Center in Pixels
c=np.matrix([pixels.item(0)/2,pixels.item(1)/2])
c=c.T

#--main-program--#
# fist labelpoint is initialized
labelpoint=np.zeros((3,n_o),dtype=int)
# labels are initialized
label=np.zeros((n_o,n_p),dtype=int)
# for loop over every object
for j in range(n_o):
	# for loop over every point
	distalt=999.0
	for i in range(n_p):
		# transform to pixel coordinates
		Xu=coordtransform(i)
		# label it with boolean
		label[j,i]=labelling(j,i)
		# change label to class number
		label[j,i]=label[j,i]*classes[j]	
		# check if it is the center point
		[lp_v,distalt]=text_label(j,i,distalt)
		if lp_v==-1:
			# do nothing if not
			lp_v=lp_v
		else:
			# change lp value if yes
			lp=lp_v
	# fill the labelpoint for every obejct in an array
	labelpoint[:,j]=points[:,lp]

		


print (labelpoint)
print (label)


	

