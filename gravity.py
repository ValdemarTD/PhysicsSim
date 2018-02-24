def main():
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
                fgrav = -g*actor.m*actee.m*dist/(mag(dist)**3)
                #Calculates acceleration on object
                actee.a = fgrav/actee.m
                #Changes object velocity
                actee.v = actee.v + actee.a * dt
        #Changes position based on summed velocity changes        
        actee.pos = actee.pos + actee.v * dt
