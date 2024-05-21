import roboticstoolbox as rtb
import numpy as np 
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH

## Create Model
# link lengths in mm
a1 = float(20)
a2 = float(20)
a3 = float(20)




# limit of variable q1
def deg_to_rad(T):
    return (T/180)*np.pi

#Create links
#robot_variable = DHRobot([RevoluteDH(d,r,alpha,offset=theta,qlim)])
#robot_variable = DHRobot([PrismaticDH(d=0,r,alpha,offset=d,qlim)])


#Create Links
Cylindrical = DHRobot([
        RevoluteDH(a1/100,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
        PrismaticDH(0,0,(270/180.0)*np.pi,a2/100,qlim=[0,(30/100)]),
        PrismaticDH(0,0,(0.0/180.0)*np.pi,a3/100,qlim=[0,(30/100)]),

    ], name="Cylindrical")
print("Cylindrical")

##Path and Trajectory Planning
#q paths
#Cylindrical Joint Variables


q0=np.array([0,0,0,]) #origin

q1=np.array([deg_to_rad(float(0)),
             (float(20)/100),
             (float(0)/100),]
             )


q2=np.array([deg_to_rad(float(0)),
             (float(20)/100),
             (float(40)/100),]
             )


q3=np.array([deg_to_rad(float(-10)),
             (float(20))/100,
             (float(50))/100,]
             )


q4=np.array([deg_to_rad(float(-10)),
             (float(50))/100,
             (float(50))/100,]
             )

            


q5=np.array([deg_to_rad(float(10)),
             (float(50))/100,
             (float(50))/100,]
             )


q6=np.array([deg_to_rad(float(10)),
             (float(20))/100,
             (float(50))/100,]
             )




q7=np.array([deg_to_rad(float(0)),
             (float(20)/100),
             (float(40)/100),]
             )





#Trajectory
traj0=rtb.jtraj(q0,q0,5)
traj1=rtb.jtraj(q0,q1,5)
traj2=rtb.jtraj(q1,q2,5)
traj3=rtb.jtraj(q2,q3,5)
traj4=rtb.jtraj(q3,q4,5)
traj5=rtb.jtraj(q4,q5,5)
traj6=rtb.jtraj(q5,q6,5)
traj7=rtb.jtraj(q6,q7,5)
traj8=rtb.jtraj(q7,q0,5)

x1 = -1
x2 = 1
y1 = -1
y2 = 1
z1 = 0.0
z2 = 1     


#PLot command
Cylindrical.plot(traj0.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj4.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj5.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj6.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj7.q,limits=[x1,x2,y1,y2,z1,z2])
Cylindrical.plot(traj8.q,limits=[x1,x2,y1,y2,z1,z2])



