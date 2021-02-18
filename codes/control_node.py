#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32
from collections import deque
import numpy as np
import sys
sys.path.append('../../')
import do_mpc
from casadi import *
import matplotlib.pyplot as plt

def get_bezier_coef(points):
    n = len(points) - 1

    # build coefficents matrix
    C = 4 * np.identity(n)
    np.fill_diagonal(C[1:], 1)
    np.fill_diagonal(C[:, 1:], 1)
    C[0, 0] = 2
    C[n - 1, n - 1] = 7
    C[n - 1, n - 2] = 2

    # build points vector
    P = [2 * (2 * points[i] + points[i + 1]) for i in range(n)]
    P[0] = points[0] + 2 * points[1]
    P[n - 1] = 8 * points[n - 1] + points[n]

    # solve system, find a & b
    A = np.linalg.solve(C, P)
    B = [0] * n
    for i in range(n - 1):
        B[i] = 2 * points[i + 1] - A[i + 1]
    B[n - 1] = (A[n - 1] + points[n]) / 2

    return A, B

# returns the general Bezier cubic formula given 4 control points
def get_cubic(a, b, c, d):
    return lambda t: np.power(1 - t, 3)*a + 3*np.power(1 - t, 2)*t*b + 3*(1 - t)*np.power(t, 2)*c + np.power(t, 3)*d

# return one cubic curve for each consecutive points
def get_bezier_cubic(points):
    A, B = get_bezier_coef(points)
    return [
        get_cubic(points[i], A[i], B[i], points[i + 1])
        for i in range(len(points) - 1)
    ]

def derivative_bezier(a,b,c,d):
    return lambda t: np.power(1-t,2)*a*(-3)+3*b*(np.power(1-t,2)-2*t*(1-t))+3*c*(2*t*(1-t)-np.power(t,2))+3*d*np.power(t,2)

def derivative_list(points):
    A,B=get_bezier_coef(points)
    return [derivative_bezier(points[i],A[i],B[i],points[i+1]) for i in range(len(points)-1)]
    
# evalute each cubic curve on the range [0, 1] sliced in n points
def evaluate_bezier(points, n):
    curves = get_bezier_cubic(points)
    return np.array([fun(t) for fun in curves for t in np.linspace(0, 1, n)])
def trajectory_gen(points):
    A,B=get_bezier_coef(points)
    return [get_cubic(points[i],A[i],B[i],points[i+1]) for i in range(len(points)-1)]

def control(x_0,i,x,y,vel,points,curves,derivatives,velocities,acc_pub,steer_rate_pub):
    model_type='discrete'
    model=do_mpc.model.Model(model_type)
    J=1000
    La=1
    Lb=1
    m=200
    Cy=0.1
    t_s=0.01 #sample time
    N=70
    k=0.1
    fn=curves[i]
    d=derivatives[i]
    vmax_i=max(velocities[i],velocities[i+1])
    #state variables
    psi=model.set_variable(var_type='_x',var_name='psi',shape=(1,1))
    xc=model.set_variable(var_type='_x',var_name='xc',shape=(1,1))
    yc=model.set_variable(var_type='_x',var_name='yc',shape=(1,1))
    v=model.set_variable(var_type='_x',var_name='v',shape=(1,1))
    theta=model.set_variable(var_type='_x',var_name='theta',shape=(1,1))
    phi=model.set_variable(var_type='_x',var_name='phi',shape=(1,1))
    delta=model.set_variable(var_type='_x',var_name='delta',shape=(1,1))
    a_s=model.set_variable(var_type='_x',var_name='a_s',shape=(1,1))
    w_s=model.set_variable(var_type='_x',var_name='w_s',shape=(1,1))
    #k=model.set_variable(var_type='_p',var_name='k',shape=(1,1))
    Fyf=Cy*(delta-(La*phi)/v)
    Fyr=(Cy*Lb*phi)/v
    #control inputs
    a=model.set_variable(var_type='_u',var_name='a',shape=(1,1))
    omega=model.set_variable(var_type='_u',var_name='omega',shape=(1,1))
    #model.set_expression(expr_name='cost', expr=sum1((xc-fn(psi)[0])**2+100*(yc-fn(psi)[1])**2+100*theta**2+200*(np.tan(theta)-d(psi)[1]/d(psi)[0])**2
                                                     #+a_s**2+w_s**2+(v-0.9*vmax_i)**2))
    model.set_expression(expr_name='cost', expr=sum1((xc-fn(psi)[0])**2+100*(yc-fn(psi)[1])**2+100*theta**2+200*(np.tan(theta)-d(psi)[1]/d(psi)[0])**2
                                                     +a_s**2+w_s**2)+(v-0.8*velocities[i+1])**2)
    state_now=vertcat(psi,xc, yc, v, theta, phi, delta,a_s,w_s)
    #B=t_s*vertcat((0.9*vmax_i)/((d(psi)[0])**2+(d(psi)[1])**2)**0.5,v*np.cos(theta), v*np.sin(theta), a* np.cos(delta)-(2.0/m)*Fyf*np.sin(delta), phi,
     #             (1.0/J)*(La*(m*a*np.sin(delta)+2*Fyf*np.cos(delta))-2*Lb*Fyr), omega,(1/t_s)*(a-a_s),(1/t_s)*(omega-w_s))
    B=t_s*vertcat(k,v*np.cos(theta), v*np.sin(theta), a* np.cos(delta)-(2.0/m)*Fyf*np.sin(delta), phi,
                  (1.0/J)*(La*(m*a*np.sin(delta)+2*Fyf*np.cos(delta))-2*Lb*Fyr), omega,(1/t_s)*(a-a_s),(1/t_s)*(omega-w_s))
    state_next=state_now + B
    model.set_rhs('psi',state_next[0])
    model.set_rhs('xc',state_next[1])
    model.set_rhs('yc',state_next[2])
    model.set_rhs('v',state_next[3])
    model.set_rhs('theta',state_next[4])
    model.set_rhs('phi',state_next[5])
    model.set_rhs('delta',state_next[6])
    model.set_rhs('a_s',state_next[7])
    model.set_rhs('w_s',state_next[8])
    model.setup()
    mpc=do_mpc.controller.MPC(model)
    setup_mpc = {
        'n_horizon': N,
        't_step': t_s,
        'n_robust': 1,
        'state_discretization':'discrete',
        'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)
    mterm = model.aux['cost']
    lterm = model.aux['cost']
    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(a=0.1)
    mpc.set_rterm(omega=0.1)
    # 'tube' from path planning
    #mpc.bounds['lower','_x','xc']=x_i[0]-1e-19
    mpc.bounds['lower','_x','yc']=min(points[i+1][1],x_0[2])-1.5
    mpc.bounds['lower','_x','v']=0 #max reverse speed in m/s
    mpc.bounds['lower','_x','theta']=-np.pi/2-1e-2
    #mpc.bounds['upper','_x','xc']=target+5
    mpc.bounds['upper','_x','yc']=max(points[i+1][1],x_0[2])+1.5
    mpc.bounds['upper','_x','theta']=np.pi/2 + 1e-2
    mpc.bounds['upper','_x','v']=vmax_i
    #mpc.bounds['upper','_x','phi']=50
    '''
    mpc.bounds['upper','_x','delta']=50
    mpc.bounds['lower','_u','a']=-10
    mpc.bounds['lower','_u','omega']=-10
    mpc.bounds['upper','_u','a']=10
    mpc.bounds['upper','_u','omega']=10
    mpc.bounds['lower','_x','a_s']=-10
    mpc.bounds['lower','_x','w_s']=-10
    mpc.bounds['upper','_x','a_s']=10
    mpc.bounds['upper','_x','w_s']=10
    '''
    mpc.setup()
    #estimator=do_mpc.estimator.StateFeedback(model)
    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = t_s)
    '''p_template=simulator.get_p_template()
    def p_fun(t_now):
        p_template['k']=0.1
        return p_template
    simulator.set_p_fun(p_fun)'''
    simulator.setup()
    mpc.x0 = x_0
    simulator.x0 = x_0
    mpc.u0['a']=x_0[-2]
    mpc.u0['omega']=x_0[-1]
    mpc.set_initial_guess()
    mpc.reset_history()
    N_u=N
    for j in range(N_u):
        u0=mpc.make_step(x_0)
        acc_pub.publish(u0[0][0])
        steer_rate_pub.publish(u0[1][0])
        x_0=simulator.make_step(u0)
        # with open('control_outputs.csv',mode='a') as op_file:
        #     op=csv.writer(op_file,delimiter=',')
        #     op.writerow([u0[0][0],u0[1][0]])
        
        if x_0[1]>=points[i+1][0]:
            break
        # x_0=estimator.make_step(y_0)
    theta=vertcat(theta,simulator.data['_x','theta',-1])
    x=vertcat(x,simulator.data['_x','xc',-1])
    y=vertcat(y,simulator.data['_x','yc',-1])
    vel=vertcat(vel,simulator.data['_x','v',-1])
    #z=vertcat(z,simulator.data['_x','theta',-1])
    if x_0[1]>=points[i+1][0]:
        return x_0,x,y,vel
    else:
        #x_0=simulator.data['_x'][-1]
        return control(x_0,i,x,y,vel,points,curves,derivatives,velocities,acc_pub,steer_rate_pub)


points=np.zeros((30,2))
velocities=np.zeros((30,1))
#for j in range(30):
 #   velocities[j][0]=1.5
'''with open("coordinates.csv") as csv_file:
  csv_reader=csv.reader(csv_file,delimiter=',')
  j=0
  for row in csv_reader:
     if j==0:
         offset_x=float(row[1])
         offset_y=float(row[0])
     points[j][0]=float(row[1])-offset_x
     points[j][1]=float(row[0])-offset_y
     j+=1
     if j>=30:
       break'''
x_dq=deque([])
y_dq=deque([])
v_dq=deque([])
x_l=[]
y_l=[]
x_p=[]
y_p=[]
def x_callback(x_c):
    x_dq.append(x_c.data)
    x_l.append(x_c.data)

def y_callback(y_c):
    y_dq.append(y_c.data)
    y_l.append(y_c.data)

def v_callback(v_max):
    v_dq.append(v_max.data)

def move_forward(x_0,acc_pub,steer_rate_pub):
    for j in range(30):
        points[j][0]=x_dq[j]-x_l[0]
        points[j][1]=y_dq[j]-y_l[0]
        velocities[j][0]=v_dq[j]
    bcurves=trajectory_gen(points)
    derivatives=derivative_list(points)
    #t_i=np.arctan(derivatives[0](0)[1]/derivatives[0](0)[0])
    #x_0=np.array([[0],[points[0][0]],[points[0][1]],[1.5],[t_i],[0],[0],[0],[0]])
    x=np.array(x_0[1])
    y=np.array(x_0[2])
    v=np.array(x_0[3])
    (x_0,x,y,v)=control(x_0,0,x,y,v,points,bcurves,derivatives,velocities,acc_pub,steer_rate_pub)
    x_dq.popleft()
    y_dq.popleft()
    v_dq.popleft()
    return x_0,x,y,v,points[0][0],points[0][1]

x_0=np.array([[0],[points[0][0]],[points[0][1]],[1.5],[0],[0],[0],[0],[0]]) #assuming initially straight path
x_=np.array([0])
y_=np.array([0])
rospy.init_node('control_node', anonymous=True)
acc_pub = rospy.Publisher('acceleration', Float32, queue_size=10)
steer_rate_pub = rospy.Publisher('steer_rate', Float32, queue_size=10)
rate=rospy.Rate(10)
while not rospy.is_shutdown():
    rospy.Subscriber("x_c",Float32,x_callback)
    rospy.Subscriber("y_c",Float32,y_callback)
    rospy.Subscriber("v_max",Float32,v_callback)
    if len(x_dq)>=30 and len(y_dq)>=30:
        x_0,x,y,v,xp,yp=move_forward(x_0,acc_pub,steer_rate_pub)
        x_=vertcat(x_,x)
        x_p.append(xp)
        y_=vertcat(y_,y)
        y_p.append(yp)
    rate.sleep()
plt.plot(x_,y_)
plt.scatter(x_p,y_p)
plt.show()


'''bcurves=trajectory_gen(points)
derivatives=derivative_list(points)
t_i=np.arctan(derivatives[0](0)[1]/derivatives[0](0)[0])
x_0=np.array([[0],[points[0][0]],[points[0][1]],[1.9],[t_i],[0],[0],[0],[0]])
x=np.array([points[0][0]])
y=np.array([points[0][1]])
v=np.array([1.9])
theta=np.array([t_i])


for i in range(len(points)-1):
    (x_0,x,y,v,theta)=control(x_0,i,x,y,v,theta,points,bcurves,derivatives,velocities)
    x_0[0]=0
path=evaluate_bezier(points,50) 
px=path[:,0]
py=path[:,1]
fig,(ax1,ax2)=plt.subplots(2,1,sharex=True)
ax1.plot(px,py)
ax1.plot(px,py-1.5)
ax1.plot(px,py+1.5)
ax1.plot(points[:,0],points[:,1],'ro')
ax1.plot(x,y)
ax2.plot(x,v)
ax2.plot(points[:,0],velocities[:,0],'ro')
ax2.plot(points[:,0],0.8*velocities[:,0],'bo')
ax1.set(xlabel='x',ylabel='y')
ax2.set(xlabel='x',ylabel='v')
plt.show()'''

# if __name__ == '__main__':
#     try:
#         for i in range(len(points)-1):
#             (x_0,x,y,v,theta)=control(x_0,i,x,y,v,theta,points,bcurves,derivatives,velocities)
#             x_0[0]=0
#         path=evaluate_bezier(points,50)
#         control_output()
#     except rospy.ROSInterruptException:
#         pass


















'''def control_output():
    acc_pub = rospy.Publisher('acceleration', Float32, queue_size=10)
    steer_rate_pub = rospy.Publisher('steer_rate', Float32, queue_size=10)
    rospy.init_node('control_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        with open('control_outputs.csv','r') as read_file:
            reader=csv.reader(read_file,delimiter=',')
            for row in reader:
                acc=float(row[0])
                sa_r=float(row[1])
                acc_pub.publish(acc)
                steer_rate_pub.publish(sa_r)
                rate.sleep()

if __name__ == '__main__':
    try:
        control_output()
    except rospy.ROSInterruptException:
        pass'''