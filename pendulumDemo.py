from visual import *
import numpy as np

g=6.67408e-11
t = 0
dt = .001

universe = display(center = vector(0,.5,0), background = (51/255/1.3, 102/255/1.3, 255/255/1.3))

earth = sphere(m = 5.972e24, radius = 6.371e6, pos = vector(0,-6.371e6, 0), v = vector(0,0,0), a = vector(0,0,0), name = "earth", label = 0, color = color.blue)
ball = sphere(m = .091, radius = .05, pos = vector(0 , 1 - .2032 , .4572 ), v = vector(0,0,0), a = vector(0,0,0), name = "ball", label = 0, color = color.red, make_trail = True, duration = 3000)
ball.trail_object.color = color.black


bodies = [earth, ball]
forces = []
joints = []

i = 0
for body in bodies:
    body.label = i
    forces.append(body)
    body.f = vector(0,0,0)
    forces[body.label] = body.f
    i = i + 1
    body.fixed = False

def fixed(pos):
    body = sphere(radius = .1, color = color.black, pos = pos, fixed = "True")
    return(body)

point = fixed(vector(0,1,0))

def Joint(body1, body2, joints, joint_type, k, elas):
        length = (body2.pos - body1.pos)
        if joint_type == "rigid":
            body = helix(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        elif joint_type == "rope":
            body = cylinder(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        joints.append(body)
        body.pos = body1.pos
        return(body)

spring = Joint(ball, point, joints, "rope", 10000, 1)

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
    rate(1000)
    t = t + dt
    Modules(modules,bodies,forces,joints)
