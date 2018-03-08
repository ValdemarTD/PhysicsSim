from visual import *
import numpy as np

g=6.67408e-11
t = 0
dt = .001

universe = display()

#Defining starting conditions for our celestial bodies
sun=sphere(name="sun",radius=5e10,color=color.yellow,pos=vector(0,0,0),m=1.981e30,a=vector(0,0,0),v=vector(0,0,0),make_trail=True,mark = 0)

mercury=sphere(name="mercury",radius=6e8,color=color.red,pos=vector(0,0,5.834e10),m=3.285e23,a=vector(0,0,0),v=vector(4.789e4,0,0),make_trail=True)

venus=sphere(name="venus",radius=6e8,color=color.green,pos=vector(0,0,1.077e11),m=4.867e24,a=vector(0,0,0),v=vector(3.503e4,0,0),make_trail=True)

earth=sphere(name="earth",radius=6e7,color=color.blue,pos=vector(0,0,1.496e11),m=5.972e24,a=vector(0,0,0),v=vector(2.979e4,0,0),make_trail=True, mark = 1)
luna=sphere(name="luna",radius=5e7,pos=vector(0,0,3.85e8)+earth.pos,m = 7.348e22,a=vector(0,0,0),v=vector(1000,0,0)+earth.v,make_trail=True, mark = 2)

mars=sphere(name="mars",radius=6e6,color=color.red,pos=vector(0,0,2.274e11),m=6.39e23,a=vector(0,0,0),v=vector(2.413e4,0,0),make_trail=True)
phobos=sphere(name="phobos",radius=5e5,pos=mars.pos+vector(0,0,9.376e6),m=10.6e15,a=vector(0,0,0),v=mars.v+vector(2138,0,0),make_trail=True)
deimos=sphere(name="deimos",radius=5e5,pos=mars.pos+vector(0,0,2.346e7),m=1.4762e15,a=vector(0,0,0),v=mars.v+vector(1351,0,0),make_trail=True)

jupiter=sphere(name="jupiter",radius=8e7,color=color.orange,pos=vector(0,0,7.779e11),m=1.898e27,a=vector(0,0,0),v=vector(1.306e4,0,0),make_trail=True)
io=sphere(name="io",radius=5e7,color=color.yellow,pos=jupiter.pos+vector(0,0,3.5e8),m=8.9319e22,a=vector(0,0,0),v=jupiter.v+vector(17334,0,0),make_trail=True)
europa=sphere(name="europa",radius=5e7,color=color.white,pos=jupiter.pos+vector(0,0,6.709e8),m=4.7998e22,a=vector(0,0,0),v=jupiter.v+vector(13740,0,0),make_trail=True)
ganymede=sphere(name="ganymede",radius=5e7,color=color.red,pos=jupiter.pos+vector(0,0,1.07e9),m=1.48e23,a=vector(0,0,0),v=jupiter.v+vector(10880,0,0),make_trail=True)
callisto=sphere(name = "callisto",radius=5e7,color=color.green,pos=jupiter.pos+vector(0,0,1.88e9),m=1.08e23,a=vector(0,0,0),v=jupiter.v+vector(8204,0,0),make_trail=True)

saturn=sphere(name="saturn",radius=7e8,color=color.green,pos=vector(0,0,1.427e12),m=5.683e26,a=vector(0,0,0),v=vector(9.64e3,0,0),make_trail=True)
titan=sphere(name="titan",radius=5e7,color=color.green,pos=saturn.pos+vector(0,0,1.222e9),m=1.3452e23,a=vector(0,0,0),v=saturn.v+vector(5570,0,0),make_trail=True)

uranus=sphere(name="uranus",radius=6.5e8,color=color.blue,pos=vector(0,0,2.869e12),m=8.681e25,a=vector(0,0,0),v=vector(9.64e3,0,0),make_trail=True)

neptune=sphere(name="neptune",radius=6.5e8,color=color.blue,pos=vector(0,0,4.479e12),m=1.024e26,a=vector(0,0,0),v=vector(5.43e3,0,0),make_trail=True)

#Note, pluto's orbit isn't fully realistic. I didn't know the exact angle to use. WIll try to fix at some point.
pluto=sphere(name="pluto",radius=4e4,color=color.white,pos=vector(0,0,5.9e12),m=1.309e22,a=vector(0,0,0),v=vector(4.74e3,2e3,0),make_trail=True)
charon=sphere(name="charon",radius=5e4,pos=pluto.pos+vector(0,0,1.964e7),m=8.958e20,a=vector(0,0,0),v=pluto.v+vector(210,0,0),make_trail=True)


bodies = [sun,mercury,venus,earth,luna,mars,phobos,deimos,jupiter,io,europa,ganymede,callisto,saturn,titan,uranus,neptune,pluto,charon]
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

def Joint(body1, body2, joints, joint_type, k, elas):
        length = (body2.pos - body1.pos)
        if joint_type == "rigid":
            body = helix(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        elif joint_type == "rope":
            body = cylinder(axis = length, jtype = joint_type, bodies = [body1, body2], k = k, radius = .02, jlength = mag(length), elas = elas)
        joints.append(body)
        body.pos = body1.pos
        return(body)

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
    Modules(modules,bodies,forces,joints)
