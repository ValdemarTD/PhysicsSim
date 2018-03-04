from visual import *
import numpy as np

g=6.67408e-11
t = 0
dt = 1

universe = display(center = vector(0,0,0))

#14064015000000000000000000 kg

#moons: 3340 kg/m^3

#r1 = 5100000
#r2 = 800000
#r3 = 1000000

#m1:1857040000000000000000000 kg
#m2:7147600000000000000000
#m3:13994600000000000000000


planet = sphere(color=color.blue, m = 14064015000000000000000000, radius = 8471000, pos = vector(0,0,0), v = vector(0,0,0), a = vector(0,0,0), label = 0, name = "planet")
moon1 = sphere(color=color.red, m = 1857040000000000000000000, radius = 5100000, pos = vector(385000000,0,0), v = vector(0,0,1500), a = vector(0,0,0), label = 0, name = "moon1", make_trail = True)
moon2 = sphere(color=color.green, m = 7147600000000000000000, radius = 800000, pos = vector(40000000,0,0), v = vector(0,800,3900), a = vector(0,0,0), label = 0, name = "moon2", make_trail = True)
moon3 = sphere(color=color.yellow, m = 13994600000000000000000, radius = 1000000, pos = vector(150000000,0,0), v = vector(0,0,2000), a = vector(0,0,0), label = 0, name = "moon3", make_trail = True)

submoon1 = sphere(m = 500000000000000, radius = 230000, pos = vector(9000000,0,0) + moon1.pos, v = vector(0,3500,50) + moon1.v, a = vector(0,0,0), label = 0, name = "submoon1", make_trail = True)



planet = planet
moon1 = moon1
moon2 = moon2
moon3 = moon3
submoon1 = submoon1

bodies = [planet, moon1, moon2, moon3, submoon1]
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
    rate(1000000)
    Modules(modules,bodies,forces,joints)
    universe.center = vector(planet.radius + 500000,0,0) + planet.pos
