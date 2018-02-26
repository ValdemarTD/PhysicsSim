from visual import *
import numpy as np

g=6.67408e-11
t = 0
dt = .001

universe = display()

ball = sphere(a = vector(0,0,0), v = vector(0,0,1), pos = vector(5,0,0), m = 5e13, name = "ball")
cube = box(a = vector(0,0,0), v = vector(0,15,0), pos = vector(0,0,0), m = 5e12, name = "cube")
rect = box(a = vector(0,0,0), v = vector(0,0,20), pos = vector(0,3,0), m = 5e12, name = "rectangle")
bodies = [ball,cube,rect]
forces = []
i = 0
for body in bodies:
    body.label = i
    forces.append(body)
    body.f = vector(0,0,0)
    forces[body.label] = body.f
    i = i + 1

def Modules(modules, bodies, forces):
    for body in bodies:
        body.f = vector(0,0,0)
        forces[body.label] = body.f
    for module in modules:
        forces = module(bodies, forces)
    for body in bodies:
        body.f = forces[body.label]
        body.a = body.f/body.m
        body.v = body.v + body.a * dt
        body.pos = body.pos + body.v * dt


def gravity(bodies, forces):
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

modules = [gravity]


while True:
    rate(100)
    Modules(modules,bodies,forces)
