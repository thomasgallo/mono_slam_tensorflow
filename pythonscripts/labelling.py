import numpy as np 

n = 30					# number of points in map
# n=points.size/3			# if an array of points could be imported from somewhere
pixels=np.matrix([320,240])
T=np.matrix([0.,0.,0.])			# Position of Camera global
T=np.transpose(T)
R=np.matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])		# Rotation Matrix of Camera
c=np.matrix([pixels.item(0)/2,pixels.item(1)/2])	# Image center in pixels
c=c.T
f=35.0 					# focal length
bbt = 100				# bounding box top
bbb = 10				# bounding box bottom
bbr = 150				# bounding box right
bbl = 30				# bounding box left

i=0
label=np.zeros(n,dtype=bool)
distalt=999.0
# A while loop over all points in the point map. It is calculated, whether these points lay inside the bounding box or not. Out of the points inside the bounding box, the one closest to its center is selected.
# Since we cannot acces the CameraData from SLAM, there is no Data imported, just random numbers.
# But the calculations should be correct
while i<n:
	#X=points[:,i]					# Accesing point from array
	X=np.random.random((3,1))			# Position of Point global
	X=X*20
	# transform Global to Camera Coordinates
	XT = (R.T)*(X-T)
	# transform Camera Coordinates to Pixel coordinates
	Xu = -np.matrix([[f*XT.item(0)/XT.item(2)],[f*XT.item(1)/XT.item(2)]])+c
	
	# test if inside bounding box
	if Xu.item(0)>bbl and Xu.item(0)<bbr and Xu.item(1)>bbb and Xu.item(1)<bbt:
		# set label active
		label.itemset(i,1)
		# calculate distance to center
		center=np.matrix([bbr-bbl,bbt-bbb])
		dist=np.linalg.norm(Xu-center)
		# initialization
		if distalt==999.0:
			lp=i
			distalt=dist
		else:
		# improvement for closer points
			if dist<distalt:
				lp=i
				distalt=dist
	else:
		# set label inactive
		label.itemset(i,0)
	i=i+1;
print (label)
print (lp)
# labelpoint = points(:,lp)


	

