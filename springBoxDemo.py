from visual import *
import numpy as np
import time

g=6.67408e-11
t = 0
dt = .01

count = 4

universe = display(width = 1680, height = 1050, center = vector(1.5,1.5,1.5), range = count + 2)

bodies = []
forces = []
joints = []


for x in range(count):
    for y in range(count):
        for z in range(count):
            body = sphere(pos = vector(x,y,z), v=vector(0,0,0), a=vector(0,0,0), m = 50, radius = .1, color = color.red)
            bodies.append(body)

i = 0
for body in bodies:
    body.label = i
    forces.append(body)
    body.f = vector(0,0,0)
    forces[body.label] = body.f
    i = i + 1
    body.fixed = False
    body.name = i
    if i == 16:
        body.v = vector(.5,.5,-.5)

def fixed(pos):
    body = sphere(radius = .1, color = color.black, pos = pos, fixed = "True")
    return(body)


def Joint(body1, body2, joints, joint_type, k, elas):
        length = (body2.pos - body1.pos)
        if joint_type == "rigid":
            body = helix(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        elif joint_type == "rope":
            body = cylinder(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        joints.append(body)
        body.pos = body1.pos
        return(body)

for body1 in bodies:
    for body2 in bodies:
        if abs(mag(body1.pos - body2.pos)) <= 1.2 and body1.label != body2.label:
            j = Joint(body1, body2, joints, "rigid", 200000, 1)



def Modules(modules, bodies, forces, joints):
    for body in bodies:
        body.f = vector(0,0,0)
        forces[body.label] = body.f
    for module in modules:
        forces = module(bodies, forces, joints)
    for body in bodies:
        body.f = forces[body.label]
        body.a = body.f/body.m
        body.v = body.v + body.a * dt
        body.pos = body.pos + body.v * dt

def jointmod(bodies, forces, joints):
    for joint in joints:
        body1 = joint.bodies[0]
        body2 = joint.bodies[1]
        joint.pos = body1.pos
        joint.axis = body2.pos - body1.pos
        if joint.axis < joint.jlength and joint.joint_type == "rope":
            jforce = vector(0,0,0)
            print("Rope")
        else:
            jforce = (mag(joint.axis) - joint.jlength) * joint.k * joint.axis/mag(joint.axis) * (joint.elas)
        if body1.fixed == False:
            forces[body1.label] = forces[body1.label] + jforce
        if body2.fixed == False:
            forces[body2.label] = forces[body2.label] - jforce
    return(forces)

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

modules = [gravity,jointmod]

while True:
    rate(100)
    Modules(modules,bodies,forces,joints)
