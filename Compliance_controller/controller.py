import odrive
from odrive.enums import *
import time
import math
from math import sin,cos,pi
import numpy as np
from IPython.display import clear_output

class COMPLIANCE_CONTROLLER():
    def __init__(self,stiffness,damper):
        self=self
        self.stiffness=stiffness#unit:N/mm
        self.damper=damper#unit:N/(mm/s)
        self.X_AXIS_ANGLE=[0.177-0.25,0.041+0.25]#unit: turns
        self.LEG=80#unit:mm
        self.TORQUE_CONSTANT=3.6#unit:N*mm/(A/100)
        self.MAX_CURRENT=100
        pass

    def Jocobi(self,encoder0,encoder1):
        theta0=(encoder0-self.X_AXIS_ANGLE[0])*2*pi
        theta1=(encoder1-self.X_AXIS_ANGLE[1]-0.5)*2*pi
        J=np.array([[-self.LEG*sin(theta0),self.LEG*sin(theta1)],[self.LEG*cos(theta0),self.LEG*cos(theta1)]])
        return J

    
    def get_end_pos(self,encoder0,encoder1):
        theta0=(encoder0-self.X_AXIS_ANGLE[0])*2*pi
        theta1=(encoder1-self.X_AXIS_ANGLE[1]-0.5)*2*pi
        alpha=(theta0+theta1)/2
        beta=(theta1-theta0)/2
        x=2*self.LEG*sin(alpha)*sin(beta)
        y=2*self.LEG*sin(alpha)*cos(beta)
        return x,y
    
    def motor_pos_at(self,pos):
        x=pos[0]
        y=pos[1]
        ad=np.sqrt(x**2+y**2)

        if ad>=2*self.LEG or ad<=0 or y<0:
            raise ValueError('position out of reach')

        else:
            beta=math.acos(ad/(2*self.LEG))%math.pi
            
            if x==0:
                alpha=math.pi/2
            else:
                alpha=math.atan(y/x)%math.pi

            theta0=alpha-beta
            theta1=alpha+beta

            pos_0=self.X_AXIS_ANGLE[0]+theta0/(2*math.pi)
            pos_1=self.X_AXIS_ANGLE[1]-theta1/(2*math.pi)

            return pos_0,pos_1

    def control(self,my_drive,ref_pos,pos_record,time_record,t0):
        delta=time.monotonic()-t0
        if my_drive.axis0.controller.config.control_mode!=1 or my_drive.axis1.controller.config.control_mode!=1:
            raise RuntimeError('the motor control mode is not at the current control now')
        pos_mea=[my_drive.axis0.encoder.pos_circular,my_drive.axis1.encoder.pos_circular]
        x_now,y_now=self.get_end_pos(pos_mea[0],pos_mea[1])
        measured_end=np.array([x_now,y_now])
        J=self.Jocobi(pos_mea[0],pos_mea[1])
        if len(pos_record)<10:
            speed=np.array([0,0])
        else:
            speed=(measured_end-pos_record[-10])/(delta-time_record[-10])

        #controller
        elongation=np.array([x_now-ref_pos[0],y_now-ref_pos[1]])
        virtual_force=-1*elongation*self.stiffness-speed*self.damper
        motor_torque=np.dot(J.T,virtual_force)
        motor_input=[]
        for i in range(2):
            if motor_torque[i]>0:
                motor_input.append(min(motor_torque[i]/self.TORQUE_CONSTANT,self.MAX_CURRENT))
            else:
                motor_input.append(max(motor_torque[i]/self.TORQUE_CONSTANT,-self.MAX_CURRENT))
        
        my_drive.axis0.controller.input_torque = motor_input[0]
        my_drive.axis1.controller.input_torque = motor_input[1]

        return delta,measured_end,speed,motor_input
    
    def check(self,my_drive):
        if my_drive.axis1.error == 0:
            print('Everything is fine')
        else:
            print('Error!')
    
    def follow_trajetory(self,my_drive,finish_time,traje_function,x1,x2,show_flag=1):
        print('ready to do the compliance control')
        time.sleep(2)

        my_drive.axis0.controller.config.control_mode=1
        my_drive.axis1.controller.config.control_mode=1

        ref_point=[x1,traje_function(x1)]

        all_time=[]
        all_pos=[]
        all_input=[]
        all_speed=[]
        sample_num=0

        t0=time.monotonic()
        delta=time.monotonic()-t0

        while delta<finish_time:
            #controller
            delta,measured_end,speed,motor_input=self.control(my_drive,ref_point,all_pos,all_time,t0)

            #update the ref pos
            x_now=delta/finish_time*(x2-x1)+x1
            y_now=traje_function(x_now)
            ref_point=[x_now,y_now]

            #record data
            all_time.append(delta)
            all_pos.append(measured_end)
            all_speed.append(speed)
            all_input.append(motor_input)

            #display the data
            sample_num+=1
            if sample_num%10==0 and show_flag==1:
                clear_output()
                print('STIFFNESS:{} \nDAMPER: {} \nORIGIN_POSITION:{} \nMEASURED POSITION: {} \nVIRTUAL ELASTIC FORCE:{} \nMOTOR INPUT: {} \nCONTROL FREQUENCTY: {}'
                    .format(self.stiffness,self.damper,np.around(ref_point,decimals=2),np.around(measured_end,decimals=2)
                            ,self.stiffness*measured_end,motor_input,sample_num/delta))
                print('SPEED: {}'.format(speed))
                self.check(my_drive)

        my_drive.axis0.controller.input_torque = 0
        my_drive.axis1.controller.input_torque = 0

        return all_time,all_pos,all_input,all_speed


class POSITION_CONTROLLER():
    def __init__(self):
        self=self
        self.X_AXIS_ANGLE=[0.177-0.25,0.041+0.25]#unit: turns
        self.LEG=80#unit:mm
        self.TORQUE_CONSTANT=3.6#unit:N*mm/(A/100)

    def get_end_pos(self,encoder0,encoder1):
        theta0=(encoder0-self.X_AXIS_ANGLE[0])*2*pi
        theta1=(encoder1-self.X_AXIS_ANGLE[1]-0.5)*2*pi
        alpha=(theta0+theta1)/2
        beta=(theta1-theta0)/2
        x=2*self.LEG*sin(alpha)*sin(beta)
        y=2*self.LEG*sin(alpha)*cos(beta)
        return x,y

    def motor_pos_at(self,pos):
        x=pos[0]
        y=pos[1]
        ad=np.sqrt(x**2+y**2)

        if ad>=2*self.LEG or ad<=0 or y<0:
            raise ValueError('position out of reach')

        else:
            beta=math.acos(ad/(2*self.LEG))%math.pi
            
            if x==0:
                alpha=math.pi/2
            else:
                alpha=math.atan(y/x)%math.pi

            theta0=alpha-beta
            theta1=alpha+beta

            pos_0=self.X_AXIS_ANGLE[0]+theta0/(2*math.pi)
            pos_1=self.X_AXIS_ANGLE[1]-theta1/(2*math.pi)

            return pos_0,pos_1
        
    def letsgo(self,my_drive,end_pos,load_time,show_position=1):
        if my_drive.axis0.controller.config.control_mode!=3 or my_drive.axis1.controller.config.control_mode!=3:
            raise RuntimeError('the control mode is not at position control mode')
            
        t0=time.monotonic()
        delta=time.monotonic()-t0
        sample_num=0
        end_motor_angle=self.motor_pos_at(end_pos)
        start_motor_angle=[my_drive.axis0.controller.pos_setpoint,my_drive.axis1.controller.pos_setpoint]
        load_speed_0=(end_motor_angle[0]-start_motor_angle[0])/load_time
        load_speed_1=(end_motor_angle[1]-start_motor_angle[1])/load_time

        while delta<load_time:
            delta=time.monotonic()-t0
            sample_num+=1

            pos_input_0=start_motor_angle[0]+load_speed_0*delta
            pos_input_1=start_motor_angle[1]+load_speed_1*delta

            my_drive.axis0.controller.input_pos = pos_input_0
            my_drive.axis1.controller.input_pos = pos_input_1

            if sample_num%20==0 and show_position==1:
                pos_mea_0=my_drive.axis0.encoder.pos_circular
                pos_mea_1=my_drive.axis1.encoder.pos_circular
                clear_output()

                print('POSITION INPUT \nAXIS0:{:.2f} degree \nAXIS1:{:.2f} degree'.format(pos_input_0*360,pos_input_1*360))
                print('*'*20)
                print('POSITION MEASURED \nAXIS0:{:.2f} degree \n AXIS1:{:.2f} degree'.format(pos_mea_0*360,pos_mea_1*360))

        print('control frequecy=',sample_num/load_time)

    def follow_traj(self,my_drive,finish_time,traje_function,x1,x2):
        all_pos=[]
        all_time=[]
        all_input=[]

        controller0=POSITION_CONTROLLER()

        print('going to the begin point')
        controller0.letsgo(my_drive,[x1,traje_function(x1)],2)
        clear_output()
        print('ready to go')
        time.sleep(1)
        print('following the given trajectory with position control')
        t0=time.monotonic()
        delta=time.monotonic()-t0
        sample_num=0

        while delta<finish_time:
            delta=time.monotonic()-t0
            sample_num+=1
            x_now=delta/finish_time*(x2-x1)+x1
            y_now=traje_function(x_now)
            end_pos_aim=[x_now,y_now]
            motor_input=controller0.motor_pos_at(end_pos_aim)
            encoder=[my_drive.axis0.encoder.pos_circular,my_drive.axis1.encoder.pos_circular]
            end_pos_measured=self.get_end_pos(encoder[0],encoder[1])

            my_drive.axis0.controller.input_pos = motor_input[0]
            my_drive.axis1.controller.input_pos = motor_input[1]

            all_time.append(delta)
            all_pos.append(end_pos_measured)
            all_input.append(motor_input)

            if sample_num%20==0:
                clear_output()
                print('DESIRED END POS: {}'.format(end_pos_aim))
                print('MEASURED END POS: {}'.format(end_pos_measured))
                print('CONTROL FREQUENCY: {}:'.format(sample_num/delta))

        return all_time,all_pos,all_input
