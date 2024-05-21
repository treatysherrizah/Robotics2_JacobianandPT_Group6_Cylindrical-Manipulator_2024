
from tkinter import *
from tkinter import messagebox
from tkinter import PhotoImage
from tkinter import ttk
from PIL import Image, ImageTk
import tkinter as tk
import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
matplotlib.use("TkAgg")

#We install ttkbootstrap as external libary for tkinter
from ttkbootstrap.constants import*
import ttkbootstrap as tb


#from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH    

#Create GUI with title

mygui = tb.Window(themename="darkly")
mygui.title("Cylindrical Manipulator Calculator")
mygui.geometry("550x450")
mygui.resizable(True,True)


def reset():
    a1_E.delete(0,END)
    a2_E.delete(0,END)
    a3_E.delete(0,END)

    t1_E.delete(0,END)
    d2_E.delete(0,END)
    d3_E.delete(0,END)

    X1_E.delete(0,END)
    Y1_E.delete(0,END)
    Z1_E.delete(0,END)

def deg_rad(T):
    return (T/180)*np.pi

    
def f_k():
    #Link Lengths in cm
    a1=float(a1_E.get())
    a2=float(a2_E.get())
    a3=float(a3_E.get())

    #joint variables
    t1=float(t1_E.get())
    d2=float(d2_E.get())
    d3=float(d3_E.get())



    #convert rotation angles
    t1=(t1/180)*np.pi



    #Parametric Table
    PT=[[(0.0/180.0)*np.pi+t1,(0.0/180.0)*np.pi,0,a1],  
    [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d2],
    [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a3+d3]]   

    #HTM Formulae
    i = 0
    H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]
    i = 1
    H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    i = 2
    H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
            [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
            [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
            [0,0,0,1]]

    #print("H0_1 = ")
    #H0_1 = np.array(np.round(H0_1,3))
    #print(H0_1)

    #print("H1_2 = ")
    #H1_2 = np.array(np.round(H1_2,3))
    #print(H1_2)

    #print("H2_3 = ")
    #H2_3 = np.array(np.round(H2_3,3))
    #print(H2_3)
    H0_1 = np.matrix(H0_1) 

    H0_2=np.dot(H0_1,H1_2)
    H0_3=np.dot(H0_2,H2_3)  
    #print("H0_3 = ")
    #H0_3=(np.array(np.around(H0_3,3)))
    #print(H0_3)
        

    H0_2=np.dot(H0_1,H1_2)
    H0_3=np.dot(H0_2,H2_3)  
    H0_3=np.array(H0_3)

    X0_3 = H0_3 [0,3]
    X1_E.delete(0,END)
    X1_E.insert(0,np.around(X0_3,3))

    Y0_3 = H0_3 [1,3]
    Y1_E.delete(0,END)
    Y1_E.insert(0,np.around(Y0_3,3))

    Z0_3 = H0_3 [2,3]
    Z1_E.delete(0,END)
    Z1_E.insert(0,np.around(Z0_3,3))


    ## Jacobian Matrix

    # Jacobian Window
    J_sw = tb.Toplevel()
    J_sw.title("Velocity Calculator")
    J_sw.geometry("600x550")
    J_sw.resizable(FALSE,FALSE)

    #1. Linear / Prismatic Vectors
    Z_1 = [[0],[0],[1]] # The [0,0,1] vector

    #Row 1 to 3, Column 1
    J1a = [[1,0,0],
           [0,1,0],
           [0,0,1]]
    J1a = np.dot(J1a,Z_1)
    J1a = np.matrix(J1a)

    J1b_1 = H0_3[0:3,3:]
    J1b_1 = np.matrix(J1b_1)
    J1b = J1b_1


    J1 = [[(J1a[1,0]*J1b[2,0])-(J1a[2,0]*J1b[1,0])],
          [(J1a[2,0]*J1b[0,0])-(J1a[0,0]*J1b[2,0])],
          [(J1a[0,0]*J1b[1,0])-(J1a[1,0]*J1b[0,0])]]

    J1 = np.matrix(J1)


    #Row 1 to 3, Column 2

    J2 = H0_1[0:3,0:3] #R0_1
    J2 = np.dot(J2,Z_1)
    J2=np.matrix(J2)


    #Row 1 to 3, Column 3

    J3 = H0_2[0:3,0:3]  #R0_2
    J3 = np.dot(J3,Z_1)
    J3 = np.matrix(J3)

    #Row 4 to 6, Column 1

    J4 = J1a
    J4 = np.matrix(J4)


    #Row 4 to 6, Column 2

    J5=[[0],[0],[0]]
    J5 = np.matrix(J5)

    #Row 4 to 6, Column 3

    J6=[[0],[0],[0]]
    J6 = np.matrix(J6)

    #Concatenated Jacobian Matrix
    JM1 = np.concatenate((J1,J2,J3),1)
    JM2 = np.concatenate((J4,J5,J6),1)

    J = np.concatenate((JM1,JM2),0)
    J=(np.array(np.around(J,4)))
    print("J  = ")
    print(J)

    ## Singularity
    D_J = np.linalg.det(JM1)
    print("D_J = ",D_J)

    ## Inverse Velocity
    I_V = np.linalg.inv(JM1)
    print("I_V = ",I_V)

    def update_velo():
            

            x_entry.configure(state=tb.NORMAL) 
            y_entry.configure(state=tb.NORMAL) 
            z_entry.configure(state=tb.NORMAL) 
            ωx_entry.configure(state=tb.NORMAL) 
            ωy_entry.configure(state=tb.NORMAL) 
            ωz_entry.configure(state=tb.NORMAL) 


            T1p = deg_rad(T1_slider.amountusedvar.get())
            d2p = d2_slider.amountusedvar.get()
            d3p = d3_slider.amountusedvar.get()

            q = np.array([[T1p],[d2p],[d3p]])
            E = np.dot(J,q)

            xp_e = E[0,0]
            x_entry.delete(0,END)
            x_entry.insert(0,str(xp_e))                
            x_entry.configure(state=tb.READONLY) 

            yp_e = E[1,0]
            y_entry.delete(0,END)
            y_entry.insert(0,str(yp_e))
            y_entry.configure(state=tb.READONLY) 


            zp_e = E[2,0]
            z_entry.delete(0,END)
            z_entry.insert(0,str(zp_e))
            z_entry.configure(state=tb.READONLY) 


            ωx_e = E[3,0]
            ωx_entry.delete(0,END)
            ωx_entry.insert(0,str(ωx_e))
            ωx_entry.configure(state=tb.READONLY) 


            ωy_e = E[4,0]
            ωy_entry.delete(0,END)
            ωy_entry.insert(0,str(ωy_e))
            ωy_entry.configure(state=tb.READONLY) 

            ωz_e = E[5,0]
            ωz_entry.delete(0,END)
            ωz_entry.insert(0,str(ωz_e))
            ωz_entry.configure(state=tb.READONLY) 




 

# Jacobian Sliders

    JV = tb.LabelFrame(J_sw,text="Joint Velocity Vectors",bootstyle="warning",relief=tb.SUNKEN)
    JV.grid(row=0,column=0,sticky="snew",pady=20)

    T1_slider = tb.Meter(JV, bootstyle=SUCCESS,subtext=" θ1' ",subtextfont= ['Arial',16,'normal'],textfont=['Arial',20,'bold'],interactive=True,textright="° rad/s" ,metertype='semi',stripethickness=5,metersize=180
                         ,amounttotal=180,subtextstyle=SUCCESS)

    T1_slider.grid(row=0,column=1,padx=15)

    d2_slider = tb.Meter(JV, bootstyle=WARNING,subtext=" d2' ",subtextfont= ['Arial',16,'normal'],textfont=['Arial',20,'bold'], interactive=True,textright=" cm/s " ,metertype='semi',stripethickness=5,metersize=180
                         ,amounttotal=80,subtextstyle=WARNING)
    d2_slider.grid(row=0,column=2)


    d3_slider = tb.Meter(JV, bootstyle=WARNING,subtext=" d3' ",subtextfont= ['Arial',16,'normal'],textfont=['Arial',20,'bold'], interactive=True,textright=" cm/s " ,metertype='semi',stripethickness=5,metersize=180
                         ,amounttotal=80,subtextstyle=WARNING)
    d3_slider.grid(row=0,column=3,padx=15)


    EV = tb.LabelFrame(J_sw,text=" End Effector Velocity Vector ", bootstyle="success",relief=tb.SUNKEN)
    EV.grid(row=1,column=0)
    


    x_velo=tb.Label(EV,text=" x'", font=("Helvetica",20),bootstyle="light",width=2,)
    x_entry = tb.Entry(EV,bootstyle="primary",width=7,font=("Helvetica",14),state="readonly")
    x_unit = tb.Label(EV,text=" cm/s ", font=("Helvetica",20),bootstyle="inverse-primary",width=5,)

    x_velo.grid(row=0,column=0,pady=30,padx=15)
    x_entry.grid(row=0,column=1)
    x_unit.grid(row=0,column=2,padx=10)

    y_velo=tb.Label(EV,text=" y' ", font=("Helvetica",20),bootstyle="light")
    y_entry = tb.Entry(EV,bootstyle="primary",width=7,font=("Helvetica",14),state="readonly")
    y_unit = tb.Label(EV,text=" cm/s ", font=("Helvetica",20),bootstyle="inverse-primary",width=5,)


    y_velo.grid(row=1,column=0)
    y_entry.grid(row=1,column=1)
    y_unit.grid(row=1,column=2,padx=10)


    z_velo=tb.Label(EV,text=" z' ", font=("Helvetica",20),bootstyle="light")
    z_entry = tb.Entry(EV,bootstyle="primary",width=7,font=("Helvetica",14),state="readonly")
    z_unit = tb.Label(EV,text=" cm/s ", font=("Helvetica",20),bootstyle="inverse-primary",width=5,)

    z_velo.grid(row=2,column=0,pady=30)
    z_entry.grid(row=2,column=1)
    z_unit.grid(row=2,column=2,padx=10)

##########################################

    ωx_velo=tb.Label(EV,text=" ωx_velo ", font=("Helvetica",20),bootstyle="danger",width=7,)
    ωx_entry= tb.Entry(EV,bootstyle="danger",width=7,font=("Helvetica",14),state="readonly")
    ωx_unit = tb.Label(EV,text=" rad/s ", font=("Helvetica",20),bootstyle="inverse-danger",width=5,)

    ωx_velo.grid(row=0,column=3,pady=30,padx=15)
    ωx_entry.grid(row=0,column=4)
    ωx_unit.grid(row=0,column=5,padx=10)

    ωy_velo=tb.Label(EV,text=" ωy_velo ", font=("Helvetica",20),bootstyle="danger",width=7,)
    ωy_entry= tb.Entry(EV,bootstyle="danger",width=7,font=("Helvetica",14),state="readonly")
    ωy_unit = tb.Label(EV,text=" rad/s ", font=("Helvetica",20),bootstyle="inverse-danger",width=5,)

    ωy_velo.grid(row=1,column=3)
    ωy_entry.grid(row=1,column=4)
    ωy_unit.grid(row=1,column=5,padx=10)

    ωz_velo=tb.Label(EV,text=" ωz_velo ", font=("Helvetica",20),bootstyle="danger",width=7,)
    ωz_entry= tb.Entry(EV,bootstyle="danger",width=7,font=("Helvetica",14),state="readonly")
    ωz_unit = tb.Label(EV,text=" rad/s ", font=("Helvetica",20),bootstyle="inverse-danger",width=5,)

    ωz_velo.grid(row=2,column=3)
    ωz_entry.grid(row=2,column=4)
    ωz_unit.grid(row=2,column=5,padx=10)




    my_style = tb.Style()
    my_style.configure('success.Outline.Button',font=("Helvetica",50))   



    update_button=tb.Button(J_sw,text=" Update ",bootstyle="success",style="success.Outline.TButton",width=20,command=update_velo)
    update_button.grid(row=3, column=0,sticky="se",padx=15,pady=25)


    #Create Links
    Cylindrical = DHRobot([
            RevoluteDH(a1/100,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
            PrismaticDH(0,0,(270/180.0)*np.pi,a2/100,qlim=[0,(30/100)]),
            PrismaticDH(0,0,(0.0/180.0)*np.pi,a3/100,qlim=[0,(30/100)]),

        ], name="Cylindrical")
    print("Cylindrical")
        
    q0 = np.array([0,0,0]) ## origin of end effector

    q1 = np.array([deg_rad(float(t1_E.get())),
                   (float(d2_E.get()))/100,
                   float((d3_E.get()))/100,])
    
    traj1 = rtb.jtraj(q0, q1, 25)

    Cylindrical.plot(traj1.q, limits = [-.7,.7,-.7,.7,0,.7],block=True)


def i_k():

    #Inverse Kinematics Using Graphical Method

    #link lengths in cm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())

    #Position Vector in cm
    xe = float(X1_E.get())
    ye = float(Y1_E.get())
    ze = float(Z1_E.get())


    #Inverse Kinematics Solution using Graphical Method

    #Solution 1
    if xe == 0 and ye<0:
        t2=-90
        t1=(-90*np.pi)/180

    elif xe==0 and ye>0:
        t2=90
        t1=(90*np.pi)/180

    else:
        t1 = np.arctan(ye/xe)
        t2 = t1*180/np.pi


    #Solution 2
    d3=np.sqrt(ye**2+xe**2)-a3

    #Solution 3
    d2=ze-a2-a1

    t1_E.delete(0,END)
    t1_E.insert(0,np.around(t2,3))

    d2_E.delete(0,END)
    d2_E.insert(0,np.around(d2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(d3,3))

    Cylindrical = DHRobot([
            RevoluteDH(a1/100,0,(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,qlim=[-np.pi/2,np.pi/2]),
            PrismaticDH(0,0,(270/180.0)*np.pi,a2/100,qlim=[0,(30/100)]),
            PrismaticDH(0,0,(0.0/180.0)*np.pi,a3/100,qlim=[0,(30/100)]),

        ], name="Cylindrical")
    
    print("Cylindrical ")

    def deg_rad(T):
        return (T/180)*np.pi
    
    q0_2= float((d2_E.get()))/100
    
    q0_3 = float((d3_E.get()))/100

    q0 = np.array([0,0,0]) ## origin of end effector

    q1 = np.array([deg_rad(float(t1_E.get())),
                q0_2,
                q0_3,])
    
    traj1 = rtb.jtraj(q0, q1, 25)

    Cylindrical.plot(traj1.q, limits = [-.7,.7,-.7,.7,0,.7],block=True)


    
    




#Link Length and Joint Variables frame
FI = tb.LabelFrame(mygui,text="Link Frames and Joint Variables",bootstyle="success",relief=tb.SUNKEN)
FI.grid(row=0,column=0)


#Link Lengths


a1=tb.Label(FI,text=" a1 ", font=(10),bootstyle="warning")
a1.grid(row=0,column=0)

a1_E=tb.Entry(FI,width=5, font=(10),bootstyle="default")   
a1_E.grid(row=0,column=1)

cm1=tb.Label(FI,text=("cm "),font=(10),bootstyle="warning")
cm1.grid(row=0,column=2)


a2=tb.Label(FI,text=" a2 ", font=(10),bootstyle="warning")
a2.grid(row=1,column=0)


a2_E=tb.Entry(FI,width=5, font=(10),)
a2_E.grid(row=1,column=1)

cm2=tb.Label(FI,text=("cm "),font=(10),bootstyle="warning")
cm2.grid(row=1,column=2)


a3=tb.Label(FI,text=" a3 ", font=(10),bootstyle="warning")
a3.grid(row=2,column=0)

a3_E=Entry(FI,width=6, font=(10))
a3_E.grid(row=2,column=1)

cm3=tb.Label(FI,text=("cm "),font=(10),bootstyle="warning")
cm3.grid(row=2,column=2)


t1=tb.Label(FI,text=" θ1 ", font=(10),bootstyle="primary")
t1.grid(row=0,column=3)

t1_E=Entry(FI,width=5, font=(10))
t1_E.grid(row=0,column=4)

deg1=tb.Label(FI,text=(" ° "),font=(10),bootstyle="primary")
deg1.grid(row=0,column=5)

d2=tb.Label(FI,text=" d2 ", font=(10),bootstyle="primary")
d2.grid(row=1,column=3)

d2_E=Entry(FI,width=5, font=(10))
d2_E.grid(row=1,column=4)

cm5=tb.Label(FI,text=("cm "),font=(10),bootstyle="primary")
cm5.grid(row=1,column=5)

d3=tb.Label(FI,text=" d3 ", font=(10),bootstyle="primary")
d3.grid(row=2,column=3)

d3_E=Entry(FI,width=5, font=(10))
d3_E.grid(row=2,column=4)

cm6=tb.Label(FI,text=("cm "),font=(10),bootstyle="primary")
cm6.grid(row=2,column=5)


#Buttons Frame

BF = tb.LabelFrame(mygui,text="Forward and Inverse",bootstyle="success",relief=tb.SUNKEN)
BF.grid(row=1,column=0)


FK=tb.Button(BF,text="Forward ↓ ",bootstyle="success",command=f_k)
rst = tb.Button(BF,text="RESET",command=reset)
IK = tb.Button(BF,text="Inverse ↑",bootstyle="danger",command=i_k)

FK.grid(row=0, column=0,sticky="snew")
rst.grid(row=0, column=1,sticky="snew")
IK.grid(row=0, column=2,sticky="snew")

#Position Vector Frame
PV = tb.LabelFrame(mygui,text="Position Vector")
PV.grid(row=2,column=0)

X1=tb.Label(PV,text=" X1 = ")
X1_E=tb.Entry(PV,width=5)
cm9=tb.Label(PV,text=("cm "))

X1.grid(row=0,column=0)
X1_E.grid(row=0,column=1)
cm9.grid(row=0,column=2)


Y1=tb.Label(PV,text=" Y1 = ")
Y1_E=tb.Entry(PV,width=5, )
cm7=tb.Label(PV,text=("cm "))

Y1.grid(row=1,column=0)
Y1_E.grid(row=1,column=1)
cm7.grid(row=1,column=2)

Z1=tb.Label(PV,text=" Z1= ")
Z1_E=tb.Entry(PV,width=5, )
cm8=tb.Label(PV,text=("cm "))

Z1.grid(row=2,column=0)
Z1_E.grid(row=2,column=1)
cm8.grid(row=2,column=2)

img = PhotoImage(file="cylindrical.png")
img = img.subsample(1,2)
PI = Label(mygui,image=img)
PI.grid(row=1,column=3)

mygui.mainloop()
