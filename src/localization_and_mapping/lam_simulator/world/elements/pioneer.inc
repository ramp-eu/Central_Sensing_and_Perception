
# Desc: Device definitions for Activemedia robots.
# Author: Richard Vaughan, Andrew Howard,  Luis Riazuelo
# Date: 10 Jun 2002
# CVS: $Id: pioneer.inc,v 1.30 2008-01-15 01:25:42 rtv Exp $


define pioneer_base position 
(
  color "red"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  origin [-0.04 0 0 0]

  size [0.44 0.38 0.22] 
)

define omni_base position 
(
  color "red"			# Default color.
  drive "omni"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  origin [-0.05 0 0 0]

  size [0.800 0.660 0.22]
)


define tenk pioneer_base
(
  # actual size
  size [4.455 2.015 1.470] # sizes from Dok-ing web site
#  size [3.005 1.530 1.470] # sizes from Dok-ing web site
#  size [5.145 2.015 1.470] # sizes from Dok-ing web site
  #size [5.905 2.015 2] # sizes from Dok-ing web site

  # the pioneer's center of rotation is offset from its center of area
  origin [0.725 0 0 0] #ovo je ispravno, ali nesto mi ne radi s rikvercom!
#  origin [0 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 5560.0 
  # differential steering model
  drive "diff"
)

define omnirob omni_base
(  
  # simplified Body shape:
  block( 
    points 5 #11 #6
#    point[0] [-0.543405   -0.301406]
#    point[1] [0.543405   -0.301406]
#    point[2] [0.543405    0.301406]
#    point[3] [-0.543405    0.301406]
#    point[4] [-0.543405   -0.301406]
    point[0] [-0.400   -0.3300]
    point[1] [0.400   -0.3300]
    point[2] [0.400    0.3300]
    point[3] [-0.400    0.3300]
    point[4] [-0.400   -0.3300]
    z [0 0.22]
  )
)

define pioneer2dx pioneer_base
(  
  # simplified Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.2]
  )
)

define irma pioneer_base
(  
  # simplified Body shape:
  block( 
    points 6 #11 #6
    point[0] [0.1300 0.2600]
    point[1] [-0.1500 0.2600]
    point[2] [-0.4800 0.1700]
    point[3] [-0.4800 -0.1700]
    point[4] [-0.1500 -0.2600]
    point[5] [0.1300 -0.2600]
    #point[6] [0.1300 -0.0510]
    #point[7] [0.1725 -0.0510]
    #point[8] [0.1725 0.0510]
    #point[9] [0.1300 0.0510]
    #point[10] [0.1300 0.2600]
    #point[0] [0.17 0.27]
    #point[1] [-0.15 0.27]
    #point[2] [-0.48 0.17]
    #point[3] [-0.48 -0.17]
    #point[4] [-0.15 -0.27]
    #point[5] [0.17 -0.27]
    z [0 0.22]
  )
)

define mv4 tenk
(  
  # simplified Body shape:
  #left tenk wheel
  block( 
    points 4
    point[0] [-1.5025   -0.7650]
    point[1] [1.5025   -0.7650]
    point[2] [1.5025   -0.2650]
    point[3] [-1.5025    -0.2650]
    z [0 0.5]
    color "gray50"
  )
  #right tenk wheel
  block( 
    points 4
    point[0] [-1.5025   0.2650]
    point[1] [1.5025    0.2650]
    point[2] [1.5025    0.7650]
    point[3] [-1.5025    0.7650]
    z [0 0.5]
    color "gray50"
  )
  block( 
    points 4
    point[0] [-1.5025   -0.7650]
    point[1] [1.5025   -0.7650]
    point[2] [1.5025    0.7650]
    point[3] [-1.5025    0.7650]
    z [0.6 1.47]
    color "gray"
  )
  block( 
    points 4 #11 #6
    #point[0] [-1.5025   -0.7650]
    #point[1] [1.5025   -0.7650]
    point[0] [1.5025   -1.0075]
    point[1] [2.9525   -1.0075]
    point[2] [2.9525    1.0075]
    point[3] [1.5025    1.0075]
    #point[6] [1.5025    0.7650]
    #point[7] [-1.5025    0.7650]
    z [0.5 1.27]
    color "gray85"
  )
)


# a Pioneer 2 or 3 in standard configuration
define fancypioneer2dx pioneer_base
(
  # this set of blocks approximates the shape of a real Pioneer

  #  The geometry is from the Webots v5.3.0 manual. Thanks to Webots
  #  and Olivier Michel. If Stage or Gazebo do not do what you want,
  #  take a look at Webots. It's a very nice commercial simulator.
 
  # main body
  block 
  (
    points 8
    point[0] [ -0.185 -0.135 ] 
    point[1] [  0.095 -0.135 ] 
    point[2] [  0.11  -0.08  ] 
    point[3] [  0.11   0.08  ] 
    point[4] [  0.095  0.135 ] 
    point[5] [ -0.185  0.135 ]
    point[6] [ -0.215  0.1   ]
    point[7] [ -0.215 -0.1   ] 
    z [ 0.059 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ -0.135  0.136 ]
    point[1]  [ -0.185  0.136 ]
    point[2]  [ -0.223  0.101 ]
    point[3]  [ -0.248  0.054 ]
    point[4]  [ -0.258  0     ]
    point[5]  [ -0.248 -0.054 ]
    point[6]  [ -0.223 -0.101 ]
    point[7]  [ -0.185 -0.136 ]
    point[8]  [ -0.135 -0.136 ]
    z [ 0.184 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ 0.046 -0.136 ]
    point[1]  [ 0.096 -0.136 ]
    point[2]  [ 0.134 -0.101 ]
    point[3]  [ 0.159 -0.054 ]
    point[4]  [ 0.168  0     ]
    point[5]  [ 0.159  0.054 ]
    point[6]  [ 0.134  0.101 ]
    point[7]  [ 0.096  0.136 ]
    point[8]  [ 0.046  0.136 ]
    z [ 0.184 0.234 ]
  )
  
  # left wheel
  block
  (
    points 4
    point[0] [  0.083  0.177 ]
    point[1] [ -0.083  0.177 ]
    point[2] [ -0.083  0.140 ]
    point[3] [  0.083  0.140 ]
    z [0 0.165 ]
    color "gray15"
  )

  # right wheel
  block
  (
    points 4
    point[0] [  0.083  -0.14 ]
    point[1] [ -0.083  -0.14 ]
    point[2] [ -0.083  -0.177 ]
    point[3] [  0.083  -0.177 ]
    z [ 0 0.165 ]
    color "gray15"
  )

  # castor
  block
  (
    points 4
    point[3] [ -0.2475  0.012 ]
    point[2] [ -0.1825  0.012 ]
    point[1] [ -0.1825 -0.012 ]
    point[0] [ -0.2475 -0.012 ]
    z [ 0 0.065 ]
    color "gray15"
  )
 
  # lid
  block
  (
    points 22
    point[21] [  0.174 0 ]
    point[20] [  0.166 -0.056 ]
    point[19] [  0.145 -0.107 ]
    point[18] [  0.112 -0.155 ]
    point[17] [  0.064 -0.190 ]
    point[16] [  -0.074 -0.190 ]
    point[15] [  -0.096 -0.160 ]
    point[14] [  -0.151 -0.160 ]
    point[13] [  -0.2   -0.155 ]
    point[12] [  -0.236 -0.107 ]
    point[11] [  -0.256 -0.056 ]
    point[10] [  -0.264  0     ]
    point[9] [  -0.256  0.056 ]
    point[8] [ -0.236  0.107 ]
    point[7] [ -0.2    0.155 ]
    point[6] [ -0.151  0.160 ]
    point[5] [ -0.096  0.160 ]
    point[4] [ -0.074  0.190 ]
    point[3] [  0.064  0.190 ]
    point[2] [  0.112  0.155 ]
    point[1] [  0.145  0.107 ]
    point[0] [  0.166  0.056 ]
    z [ 0.234 0.24 ]

    # a dark top looks more realistic, but isn't very useful 
    # for a top-down view
    #color "gray10"
  )
)




# The Pioneer3DX standard configuration
define pioneer3dx pioneer_base
(
  # Actual size
  size [0.511 0.4 0.22 ]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0]

  # Estimated mass in KG
  mass 23.0 

  # Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )

  # Use the sonar array defined above
#  p3dx_sonar( pose [ 0 0 -0.03 0 ] )  
) 

# The Pioneer3AT standard configuration
define pioneer3at pioneer_base
(
  # Actual size
  size [0.626 0.505]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0]

  # Estimated mass in KG
  mass 40.0 

  # Body shape:
  block( 
    points 8
    point[0] [-0.18 0.313]
    point[1] [0.18 0.313]
    point[2] [0.2525 0.18]
    point[3] [0.2525 -0.18]
    point[4] [0.18 -0.313]
    point[5] [-0.18 -0.313]
    point[6] [-0.2525 -0.18]
    point[7] [-0.2525 0.18]
  )

  # Use the sonar array defined above
#  p3at_sonar( pose [ 0 0 -0.03 0 ] )  
)


### AMIGOBOT ####


define amigobot position
(
  size [0.330 0.280 0.25]
  origin [0 0 0 0] # what should this value be? send email to vaughan@sfu.ca.
#  amigo_sonar( pose [0 0 -0.02 0 ] )
)
