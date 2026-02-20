#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration


from dataclasses import dataclass

@dataclass
class node:
    x: int
    y: int
    i: int
    e: int = None


@dataclass
class cache:
    sides=[None,None,None,None]
    def clear(self):
        self.sides=[None,None,None,None]
    def check(self,d):
        f=drivetrain.heading(DEGREES)//90
        if(d==LEFT):
            f=f-1
        if(d==RIGHT):
            f=f+1
        f=f%4
        if(self.sides[f]==None):
            drivetrain.turn_for(d,90,DEGREES,wait=True)
            self.sides[f]=(distance.get_distance(MM)<SL/2)
            drivetrain.turn_for(d,-90,DEGREES,wait=True)
        return self.sides[f]
    def ahead(self):
        f=drivetrain.heading(DEGREES)//90
        self.sides[f]=(distance.get_distance(MM)<SL/2)
        return self.sides[f]

#side length of grid
SL=250#mm

def newNode():
    return node(location.position(X,MM)//100,location.position(Y,MM)//100,drivetrain.heading(DEGREES))

def main():
    nodes=[]
    top=-1
    sides=cache()
    """
    #cheap left wall
    while(not down_eye.detect(RED)):
        drivetrain.turn_for(LEFT,90,DEGREES)
        while(distance.get_distance(MM)<SL/2):
            drivetrain.turn_for(RIGHT,90,DEGREES,wait=True)
        drivetrain.drive_for(FORWARD,SL,MM)
    """

    nodes.append(newNode())
    top=top+1
    brain.print("n")

    drivetrain.turn_for(LEFT,90,DEGREES,wait=True)
    while(not down_eye.detect(RED)):
        while(distance.get_distance(MM)<SL/2):
            drivetrain.turn_for(RIGHT,90,DEGREES,wait=True)
        brain.print("r")

        #check for backtrack
        if(((drivetrain.heading(DEGREES)+180)%360)==nodes[top].i):
            brain.print("b")
            top=top-1
            nodes.pop()
            sides.clear()
            while(location.position(X,MM)//100 != nodes[top].x or location.position(Y,MM)//100 != nodes[top].y):
                drivetrain.drive_for(FORWARD,SL,MM,wait=True)
            drivetrain.turn_for(LEFT,90,DEGREES,wait=True)
            continue
        
        nodes[top].e=drivetrain.heading(DEGREES)
        b=False
        while(not b and not down_eye.detect(RED)):
            brain.print("t")
            drivetrain.drive_for(FORWARD,SL,MM)
            sides.clear()
            b=sides.ahead()
            if(not b):
                b=(not sides.check(LEFT))
            if(not b):
                b=(not sides.check(RIGHT))

        nodes.append(newNode())
        top=top+1
        brain.print("n")
        i=0
        c=False
        while(i<top and not c):
            c=nodes[top].x==nodes[i].x and nodes[top].y==nodes[i].y
            i=i+1
        if(c):
            drivetrain.turn_for(LEFT,180,DEGREES,wait=True)
            continue
        if(not sides.check(LEFT)):
            drivetrain.turn_for(LEFT,90,DEGREES,wait=True)
    #victory lap
    r=top
    while(True):
        brain.print("\nreturning to home")
        while(r>0):
            drivetrain.turn_to_heading(180+nodes[r].i,DEGREES)
            r=r-1
            while(location.position(X,MM)//100 != nodes[r].x or location.position(Y,MM)//100 != nodes[r].y):
                    drivetrain.drive_for(FORWARD,SL,MM,wait=True)
        brain.print("\nreturning to goal")
        while(r<top):
            drivetrain.turn_to_heading(nodes[r].e,DEGREES)
            r=r+1
            while(location.position(X,MM)//100 != nodes[r].x or location.position(Y,MM)//100 != nodes[r].y):
                    drivetrain.drive_for(FORWARD,SL,MM,wait=True)

vr_thread(main)
