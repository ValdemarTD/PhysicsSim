from visual import *
import numpy as np

#Unless otherwise stated, the unit system is assumed to be SI.

#Important variable initialization
g=6.67408e-11
t = 0
dt = .001
r = 100
totlen = 1.0
nodecount = 90.0
stringm = .02

universe = display(center = vector(0,1,0))


#Initialize arrays needed later
bodies = []
forces = []
joints = []


i = 0.0
while i < nodecount:
    node = sphere(radius = .005, m = stringm/nodecount, pos = vector(0,i * totlen/nodecount, 0), v = vector(0,0,0), a = vector(0,0,0), name = i)
    bodies.append(node)
    i = i + 1.0
    
#Assigns each body a number for self-interaction prevention
i = 0
for body in bodies:
    body.label = i
    forces.append(body)
    body.f = vector(0,0,0)
    forces[body.label] = body.f
    i = i + 1
    body.fixed = False

bodies[0].v.z = 10

#Allows for the creation of non-interacting objects
def fixed(pos):
    body = sphere(radius = .01, color = color.black, pos = pos, fixed = "True")
    return(body)

point = fixed(vector(0,1.0 + totlen/nodecount,0))

point.color = color.yellow
#Allows for the easy creation of joints between objects
def Joint(body1, body2, joints, joint_type, k, elas):
        length = (body2.pos - body1.pos)
#Allows multiple joint types. If you want to create another joint type, then it should be defined here with another "elif" statement.
        if joint_type == "rigid":
            body = helix(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .005, jlength = mag(length), elas = elas)
        elif joint_type == "rope":
            body = cylinder(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
#Makes sure the joint ends up in the list. No need to do it manually.
        joints.append(body)
        body.pos = body1.pos
        return(body)

for body1 in bodies:
    for body2 in bodies:
        if body1.label != body2.label and mag(body1.pos - body2.pos) <= 1.2 * totlen/nodecount:
            j = Joint(body1, body2, joints, "rigid", 100, 1)

fixedjoint = Joint(bodies[int(nodecount-1)], point, joints, "rigid", 100, 1)

#Format for joint creation should go as follows:
#NewJoint = Joint(first_body, second_body, joint_array, k_coefficient, elasticity_of_joint)

#In this case, elasticity may be the wrong term. It governs how much energy is lost as heat.
#If set to 1, no energy is lost. If set to 0, all energy is lost to heat.


#Define main module's loop. Probably won't ever need changing. 
def Modules(modules, bodies, forces, joints):
#Resets all forces for re-calculation
    for body in bodies:
        body.f = vector(0,0,0)
        forces[body.label] = body.f
#Runs every module and sets the force array equal to whatever they return.
#This is why every module must return a force array instead of simply modifying it (though, in retrospect, that may be a better way of doing it)
    for module in modules:
        forces = module(bodies, forces, joints)
#After all forces are calculated, calculates individual instantaneous accelerations, velocities, and new positions.
    for body in bodies:
        body.f = forces[body.label]
        body.a = body.f/body.m
        body.v = body.v + body.a * dt
        body.pos = body.pos + body.v * dt

#Module which governs joint interaction. Define any new joint interactions here.
def jointmod(bodies, forces, joints):
    for joint in joints:
        body1 = joint.bodies[0]
        body2 = joint.bodies[1]
        joint.pos = body1.pos
        joint.axis = body2.pos - body1.pos
        if joint.axis < joint.jlength and joint.jtype == "rope":
            jforce = vector(0,0,0)
        elif joint.jtype == "rigid":
            jforce = (mag(joint.axis) - joint.jlength) * joint.k * joint.axis/mag(joint.axis) * (joint.elas)
        if body1.fixed == False:
            forces[body1.label] = forces[body1.label] + jforce
        if body2.fixed == False:
            forces[body2.label] = forces[body2.label] - jforce
    return(forces)

#Gravitation module. Shouldn't need any tweaking.
def gravity(bodies, forces, joints):
    for actee in bodies:
        #Selects object causing action
        for actor in bodies:
            #Makes sure objects don't act upon themselves (infinite force or divide by zero errors)
            if actor.name == actee.name:
                continue
            else:
                #Checks distance between objects
                dist = actee.pos - actor.pos
                #Calculates gravitational force acting on object
                f = g*actor.m*actee.m*dist/(mag(dist)**3)
                forces[actor.label] = forces[actor.label] + f
    return(forces)

#List of all modules used. Any module not included on this list will not be included in calculations
modules = [jointmod]


#Main loop
while True:
    rate(r)
    Modules(modules,bodies,forces,joints)
    t = t + dt
    if t >= 3:
        dt = 0
