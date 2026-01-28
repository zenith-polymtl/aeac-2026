besoin d'activer l'approche avec sur 
/Mission-2/auto_approach

*Controller interface



* polar needs local pose estimation of target. Either sent by vision somehow, or sent here and relayed. For now assumes visions sends it.

* Polar needs to be adapted to output a point reached output to 


/Mission-2/in_position

Polar needs to be adapted to be able to only change 1 absolute coordinate

Once in position, the auto approach sends a aim-go to the gimbal controler, which should aim. Target is NOT sent by the conroller. It should be sent by machine vision. vision can subscribe to the aim go topic

/Mission-2/aim_topic

Once aimed finished, gimbal controller should output a finished message on 

'/Mission-2/aimed_topic'

Once aimed, the go will be given to valve on shoot_topic

* Changed shoot_topic to be valve opening command insead of just a bool.


Once valve finished shooting, the auto approach node needs to sent back the finished shoot version.

the image go should than be sent by auto_approach. 

auto approach then assumes the image is sent to gcs and waits for an approuval message. 

if approuved, the control is given back to LOITER

if not approuved, give back control to LOITER and hope next one works?

Reset all variables to start over

