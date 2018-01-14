#! /usr/bin/python

#Jeremy Lim
#This class contains the info for a single leg of the mech.

#for rendering.
import pygame
#for matrix math.
import numpy as np
#general math
import math

class MechLeg:

    def __init__(self,in_pixels_unit,in_max_range,in_angular_sweep,mech_leg_trans,mech_leg_rot,in_stand_pos,in_min_range=0):
            #Matrix defining transformation from mech to this leg.
            #(0,0) is the root of the leg.
            self.mech_to_leg = mech_leg_trans*mech_leg_rot
            #the other way
            self.leg_to_mech = np.linalg.inv(mech_leg_rot)*np.linalg.inv(mech_leg_trans)
            #NOTE!!!! ALWAYS POST-MULTIPLY!!!!

            self.pixels_per_u = in_pixels_unit


            #The Homogeneous position(in the leg's frame) the leg defaults to.
            self.stand_pos = in_stand_pos
            #our current position(initialized at standing)
            #Homogeneous
            self.position = in_stand_pos
            #How far this idealized leg can extend
            self.max_range = in_max_range
            #How close to its base it can retract
            self.min_range = in_min_range
            self.angular_sweep = in_angular_sweep

            #center of rotation, relative to stand position
            self.pivot_pos = np.matrix([0,0,0])
            #speed in radians, around the pivot position
            #+ or -
            self.radSpeed = 0.0

            #l1 homogenous end coord
            self.l1Pt = np.matrix([self.max_range*math.cos((math.pi-self.angular_sweep)/2.0),self.max_range*math.sin((math.pi-self.angular_sweep)/2.0),1.0])
            #l2 homogenous end coord
            self.l2Pt = np.matrix([-self.max_range*math.cos((math.pi-self.angular_sweep)/2.0),self.max_range*math.sin((math.pi-self.angular_sweep)/2.0),1.0])

            #Target position, in self coords.
            #Homogeneous
            self.target = self.stand_pos
            #Last given speed command. How fast we move to our target.
            #units per second.
            self.speed = 0
            #are we on the ground, or not
            self.grounded = True

    #set new target position, and/or speed.
    #robot command step
    def setTarget(self,in_position,in_speed):
        self.speed = in_speed
        self.target = in_position
        self.radSpeed = 0.0

    #constrain an angle to the 0 to 2PI range.
    def constrainAngle(self,daAngle):
        if(daAngle > 2.0*math.pi):
            return daAngle - math.floor(daAngle/(2.0*math.pi))*2.0*math.pi
        elif(daAngle < 0.0):
            return 2.0*math.pi + (daAngle - math.ceil(daAngle/(2.0*math.pi))*2.0*math.pi)
        else:
            return daAngle

    #Set a target position to be the current position, plus
    #some amount in some direction
    #only for linear movement.
    def setTargetExtend(self,in_direction,in2_speed):
        self.speed = in2_speed
        self.target = self.position+(in_direction*self.mech_to_leg)
        self.radSpeed = 0.0

    def setTargetAngleExtend(self,in_newAngle,in_radspeed,in_pivot):
        #check if we are all using the same pivot.
        if in_newAngle > 0.0:
            self.radSpeed = abs(in_radspeed)
        elif in_newAngle < 0.0:
            self.radSpeed = abs(in_radspeed)*-1.0
        else:
            self.radSpeed = 0.0 
        self.pivot_pos = in_pivot*self.mech_to_leg

        radius = np.linalg.norm(self.position-self.pivot_pos)

        self.speed = abs(radius*in_radspeed)
        #self.target = np.matrix([self.pivot_pos.item(0) + radius*math.cos(in_newAngle),self.pivot_pos.item(1) + radius*math.sin(in_newAngle),1.0])
        self.target = self.pivot_pos + (self.position - self.pivot_pos)*np.transpose(np.matrix([[math.cos(in_newAngle),-math.sin(in_newAngle),0],[math.sin(in_newAngle),math.cos(in_newAngle),0],[0,0,1]]))
        #print self.target

    def specialProjectExtend(self,in_direction,in3_speed,extend_dist):
        #project our current position onto the stand point line.
        self.speed = in3_speed
        project_pos = self.stand_pos + (np.inner((self.position-self.stand_pos),in_direction)/np.inner(in_direction,in_direction))*in_direction
        self.target = project_pos + in_direction/np.linalg.norm(in_direction)*extend_dist
        self.radSpeed = 0.0

    def goMechtoLeg(self,in_vector):
        return in_vector*self.mech_to_leg

    def goLegtoMech(self,in_vector):
        return in_vector*self.leg_to_mech

    def getLineIntersect(self,start_pt,in_direction,in_pt_a,in_pt_b):
        a_const = in_pt_a.item(1)-in_pt_b.item(1)
        b_const = in_pt_b.item(0)-in_pt_a.item(0)
        c_const = -1*(a_const*in_pt_a.item(0) + b_const*in_pt_a.item(1))
                            
        denominator = (a_const*in_direction.item(0) + b_const*in_direction.item(1))
        is_parallel = (denominator == 0.0)
        dist = -1
        if not is_parallel:
            tParam  = -1*(a_const*start_pt.item(0)+b_const*start_pt.item(1)+c_const)/denominator
            dist = tParam*np.linalg.norm(in_direction)
                                                                                            
            #see if our intersection falls within our segment.
            intersectPt = tParam*in_direction + start_pt

            is_parallel = not (intersectPt.item(0) <= max(in_pt_a.item(0),in_pt_b.item(0)) and intersectPt.item(0) >= min(in_pt_a.item(0),in_pt_b.item(0)) and intersectPt.item(1) <= max(in_pt_a.item(1),in_pt_b.item(1)) and intersectPt.item(1) >= min(in_pt_a.item(1),in_pt_b.item(1)))

        return dist, is_parallel
    

    #def getValidCircleIntersects(self,start_pt,I

    #how far a leg can extend in a given direction from
    #the current position.
    def extensionDist(self,mech_direction,in_reversed,usePosition,in_place):
        #ok, find the intersects.
        #cannot default to a member, annoyingly.
        if usePosition == False:
            in_current = self.position
        else:
            in_current = in_place
        #mech direction to leg direction.
        in_direction = (mech_direction*self.mech_to_leg)
        if in_reversed:
            in_direction = in_direction*-1

        intersectList = []
        t_param = 0
        slope = math.atan((math.pi-self.angular_sweep)/2.0)
        #left line:
        if(slope*in_direction.item(0) + in_direction.item(1) != 0):
            t_param = -1*(slope*in_current.item(0) + in_current.item(1))/(slope*in_direction.item(0) + in_direction.item(1))
            if(t_param >= 0.0):
                intersectList.append(t_param)
                #check: is real?
        #right line:
        if(-slope*in_direction.item(0) + in_direction.item(1) != 0):
            t_param = -1.0*(-slope*in_current.item(0) + in_current.item(1))/(-slope*in_direction.item(0) + in_direction.item(1))
            if(t_param >= 0.0):
                intersectList.append(t_param)

        #radius (circle)
        const_a = math.pow(in_direction.item(0),2.0)+math.pow(in_direction.item(1),2.0)
        const_b = 2.0*in_direction.item(0)*in_current.item(0) + 2.0*in_direction.item(1)*in_current.item(1) 
        const_c = math.pow(in_current.item(0),2.0)+math.pow(in_current.item(1),2.0)-math.pow(self.max_range,2.0)
        #determinant check
        det = math.pow(const_b,2.0) - 4.0*const_a*const_c
        if(det >= 0.0):
            tA = (-const_b + math.sqrt(det))/(2.0*const_a)
            tB = (-const_b - math.sqrt(det))/(2.0*const_a)
            if(tA >= 0.0):
                #check: is a real intersect?
                intersectList.append(tA)

            if(tB >= 0.0):
                #check: is a real intersect?
                intersectList.append(tB)

        #find the minimum of the intersect list.
        #pare down false intersects


        if len(intersectList) > 0:
            intersectList.sort()

            tMin = min(intersectList)
            #buffer of 0.5?
            #intersect = in_current + (tMin*0.2)*in_direction - (in_direction/np.linalg.norm(in_direction))*0.5
            intersect = in_current + tMin*in_direction - (in_direction/np.linalg.norm(in_direction))*0.5

            trueDistance = np.linalg.norm(intersect - self.position)

            #return the minimum distance found, and the intersect position
            return tMin*np.linalg.norm(in_direction),intersect,trueDistance
        else:
            trueDistance = np.linalg.norm(in_current - self.position)
            return 0.0, in_current, trueDistance

    def circularTranslation(self,in_pivot,in_position,signed_distance):
        beamVector = in_position-in_pivot
        half_diameter = np.linalg.norm(in_position-in_pivot)
        angle = math.atan2(beamVector.item(1),beamVector.item(0)) + signed_distance/half_diameter

        return np.mat([in_pivot.item(0) + half_diameter*math.cos(angle),in_pivot.item(1) + half_diameter*math.sin(angle),1.0])

    def circularExtensionDist(self,mech_direction,in_rot_point,in_reversed,usePosition,in_place,setCounterClock):
        #ok, find the intersects.
        #cannot default to a member, annoyingly.
        if usePosition == False:
            in_current = self.position
        else:
            in_current = in_place
        #mech direction to leg direction.
        in_direction = (mech_direction*self.mech_to_leg)
        pivot_point = (in_rot_point*self.mech_to_leg)

        if in_reversed:
            in_direction = in_direction*-1

        #if the rotation point is left of our direction vector, we are
        # going counter-clockwise.
        radius = np.linalg.norm(in_current - pivot_point)
        if np.linalg.norm(in_direction) != 0.0 and False:
            rotangle = -math.atan2(in_direction.item(1),in_direction.item(0))
            compareVec = (in_current - pivot_point)*np.transpose(np.matrix([[math.cos(rotangle),-math.sin(rotangle),0],[math.sin(rotangle),math.cos(rotangle),0],[0,0,1]]))

            if compareVec.item(1) >= 0: #counterclockwise
                isCounterClockwise = True
                #print "counter-clockwise"
            else:
                isCounterClockwise = False
                #print "Clockwise"
        else:
            isCounterClockwise = setCounterClock

        #if isCounterClockwise:
            #print "counter-clockwise"
        #else:
            #print "clockwise"

        intersectList = []

        #find the minimum of the intersect list.
        #pare down false intersects
        center = np.matrix([0,0,1])
        sweep = (math.pi-self.angular_sweep)/2.0
        l1pt = np.matrix([self.max_range*math.cos(sweep),self.max_range*math.sin(sweep),1.0])
        sweep = math.pi - sweep
        l2pt = np.matrix([self.max_range*math.cos(sweep),self.max_range*math.sin(sweep),1.0])

        intersectList = self.parameterizedCircleIntersect(center,self.max_range,in_current,pivot_point,(not isCounterClockwise)) + self.parameterizedlineIntersect(center,l1pt,in_current,pivot_point,(not isCounterClockwise)) + self.parameterizedlineIntersect(center,l2pt,in_current,pivot_point,(not isCounterClockwise))
        #intersectList = self.parameterizedCircleIntersect(center,self.max_range,in_current,pivot_point,(not isCounterClockwise))
        #intersectList = intersectList + self.parameterizedlineIntersect(center,l1pt,in_current,pivot_point,(not isCounterClockwise))
        #intersectList = intersectList + self.parameterizedlineIntersect(center,l2pt,in_current,pivot_point,(not isCounterClockwise))

        if len(intersectList) > 0:
            intersectList.sort()

            dMin = min(intersectList)
            dMin2 = dMin/radius
            #buffer of 0.5?
            #intersect = in_current + (tMin*0.2)*in_direction - (in_direction/np.linalg.norm(in_direction))*0.5
            #intersect = in_current + tMin*in_direction - (in_direction/np.linalg.norm(in_direction))*0.5
            if isCounterClockwise:
                intersect = self.circularTranslation(pivot_point,in_current,dMin)
            else:
                intersect = self.circularTranslation(pivot_point,in_current,-dMin)

            trueDistance = np.linalg.norm(intersect - self.position)

            #return the minimum distance found, and the intersect position
            return dMin2,intersect,trueDistance
        else:
            trueDistance = np.linalg.norm(in_current - self.position)
            return 0.0, in_current, trueDistance

    def parameterizedCircleIntersect(self,obs_center,obs_radius,leg_pt,turn_center,clockwise):
        relativeVec = leg_pt-turn_center
        radius2 = np.linalg.norm(leg_pt-turn_center)

        counterClockwiseList = []
        clockwiseList = []

        if np.linalg.norm(obs_center-turn_center) > (radius2+obs_radius):
            if (np.linalg.norm(obs_center-turn_center) + radius2) < obs_radius or (np.linalg.norm(obs_center-turn_center) + obs_radius) < radius2:
                return []
        
        xDif = turn_center.item(0)-obs_center.item(0)
        yDif = turn_center.item(1)-obs_center.item(1)
        rDif = pow(obs_radius,2.0) - pow(radius2,2.0)

        partial = (rDif - pow(xDif,2.0) - pow(yDif,2.0))/(2.0*radius2*math.sqrt(pow(xDif,2.0) + pow(yDif,2.0)))
        if partial < -1.0:
            if partial > -1.0001:
                partial = -1.0
            else:
                #no intersections
                return []
        elif partial > 1.0:
            if partial < 1.0001:
                partial = 1.0
            else:
                #no intersections
                return []
        partial = math.acos(partial)

        theta1 = partial + math.atan2(yDif,xDif)
        theta2 = math.atan2(yDif,xDif) - partial

        legConstrained = self.constrainAngle(math.atan2(relativeVec.item(1),relativeVec.item(0)))
        oppositeLimit = self.constrainAngle(legConstrained + math.pi)
        theta1 = self.constrainAngle(theta1)
        theta2 = self.constrainAngle(theta2)


        #distT1 = min(abs(theta1 - legConstrained)*radius2,abs((2.0*math.pi - theta1) + legConstrained)*radius2)
        distT1 = min(abs(theta1 - legConstrained)*radius2,(2.0*math.pi-abs(theta1 - legConstrained))*radius2)
        #distT2 = min(abs(theta2 - legConstrained)*radius2,abs((2.0*math.pi - theta2) + legConstrained)*radius2)
        distT2 = min(abs(theta2 - legConstrained)*radius2,(2.0*math.pi-abs(theta2 - legConstrained))*radius2)

        if theta1 >= legConstrained and theta1 <= oppositeLimit:
            #counterclockwise
            counterClockwiseList.append(distT1)
        elif theta1 > oppositeLimit and theta1 < legConstrained:
            #clockwise
            clockwiseList.append(distT1)
        else:#zero-crossing
            if legConstrained > math.pi:
                #counterclockwise
                counterClockwiseList.append(distT1)
            else:
                #clockwise
                clockwiseList.append(distT1)

        if theta2 >= legConstrained and theta2 <= oppositeLimit:
            #counterclockwise
            counterClockwiseList.append(distT2)
        elif theta2 > oppositeLimit and theta2 < legConstrained:
            #clockwise
            clockwiseList.append(distT2)
        else:#zero-crossing
            if legConstrained > math.pi:
                #counterclockwise
                counterClockwiseList.append(distT2)
            else:
                #clockwise
                clockwiseList.append(distT2)

        if clockwise:
            return clockwiseList
        else:
            return counterClockwiseList
        

    def parameterizedlineIntersect(self,pt_a,pt_b,leg_pt,turn_center,clockwise):
        relativeVec = leg_pt-turn_center
        radius2 = np.linalg.norm(leg_pt-turn_center)

        #negative slope, if b = 1
        if (pt_a.item(0)-pt_b.item(0)) != 0.0:
            const_a = -(pt_a.item(1)-pt_b.item(1))/(pt_a.item(0)-pt_b.item(0))
            const_b = 1.0
            const_c = -(const_a*pt_a.item(0) + const_b*pt_a.item(1))
        else:
            const_a = 1.0
            const_b = 0.0
            const_c = -pt_a.item(0)

        counterClockwiseList = []
        clockwiseList = []

        #return []
        partial = -(const_c + const_a*turn_center.item(0) + const_b*turn_center.item(1))/(radius2*math.sqrt(pow(const_a,2.0)+pow(const_b,2.0)))
        #if const_a == 0.0:
            #tangTerm = 0.0
        #else:
        if partial < -1.0:
            if partial > -1.0001:
                partial = -1.0
            else:
                #no intersections
                return []
        elif partial > 1.0:
            if partial < 1.0001:
                partial = 1.0
            else:
                #no intersections
                return []

        tangTerm = math.atan2(const_b,const_a)

        theta1 = math.acos(partial) + tangTerm
        theta2 = tangTerm - math.acos(partial)


        t1x = radius2*math.cos(theta1) + turn_center.item(0)
        t1y = radius2*math.sin(theta1) + turn_center.item(1)
        t2x = radius2*math.cos(theta2) + turn_center.item(0)
        t2y = radius2*math.sin(theta2) + turn_center.item(1)


        legConstrained = self.constrainAngle(math.atan2(relativeVec.item(1),relativeVec.item(0)))
        oppositeLimit = self.constrainAngle(legConstrained + math.pi)
        theta1 = self.constrainAngle(theta1)
        theta2 = self.constrainAngle(theta2)


        distT1 = min(abs(theta1 - legConstrained)*radius2,abs((2.0*math.pi - theta1) + legConstrained)*radius2)
        distT2 = min(abs(theta2 - legConstrained)*radius2,abs((2.0*math.pi - theta2) + legConstrained)*radius2)

        #check if our point is in the correct segment
        if (t1x - min(pt_a.item(0),pt_b.item(0))) > -0.00001 and (max(pt_a.item(0),pt_b.item(0)) - t1x) > -0.00001 and (t1y - min(pt_a.item(1),pt_b.item(1))) > -0.00001 and (max(pt_a.item(1),pt_b.item(1)) - t1y) > -0.00001:
            if theta1 >= legConstrained and theta1 <= oppositeLimit:
                #counterclockwise
                counterClockwiseList.append(distT1)
            elif theta1 > oppositeLimit and theta1 < legConstrained:
                #clockwise
                clockwiseList.append(distT1)
            else:#zero-crossing
                if legConstrained > math.pi:
                    #counterclockwise
                    counterClockwiseList.append(distT1)
                else:
                    #clockwise
                    clockwiseList.append(distT1)

        if (t2x - min(pt_a.item(0),pt_b.item(0))) > -0.00001 and (max(pt_a.item(0),pt_b.item(0)) - t2x) > -0.00001 and (t2y - min(pt_a.item(1),pt_b.item(1))) > -0.00001 and (max(pt_a.item(1),pt_b.item(1)) - t2y) > -0.00001:
            if theta2 >= legConstrained and theta2 <= oppositeLimit:
                #counterclockwise
                counterClockwiseList.append(distT2)
            elif theta2 > oppositeLimit and theta2 < legConstrained:
                #clockwise
                clockwiseList.append(distT2)
            else:#zero-crossing
                if legConstrained > math.pi:
                    #counterclockwise
                    counterClockwiseList.append(distT2)
                else:
                    #clockwise
                    clockwiseList.append(distT2)

        if clockwise:
            return clockwiseList
        else:
            return counterClockwiseList

    #put your foot down!
    def setFootState(self,in_foot):
        self.grounded = in_foot

    def getFootState(self):
        return self.grounded

    def legTipPosition(self):
        return self.position*self.leg_to_mech

    def legTipRelative(self):
        return self.position

    def getStandPos(self):
        return self.stand_pos

    def getStandPosMech(self):
        return self.stand_pos*self.leg_to_mech

    def getMechTarg(self):
        return self.target*self.leg_to_mech
    
    def stahp(self):
        self.target = self.position

    def getTargDist(self):
        return np.linalg.norm(self.target-self.position)
        
    #update state, like position and velocity.
    #simulation step
    def updateState(self,in_elapsed_time):
        #If we're close enough
        if(self.speed*in_elapsed_time > np.linalg.norm(self.target-self.position)):
            self.position = self.target
            if(self.grounded == False):
                #We've arrived: put the foot down!
                self.grounded = True

            self.speed = 0
            self.radSpeed = 0
        elif self.speed != 0.0:#otherwise, just move the position a little.
            if self.radSpeed != 0.0:
                #self.speed is the linear component, calculated...
                rotangle = self.radSpeed*in_elapsed_time
                self.position = self.pivot_pos + (self.position - self.pivot_pos)*np.transpose(np.matrix([[math.cos(rotangle),-math.sin(rotangle),0],[math.sin(rotangle),math.cos(rotangle),0],[0,0,1]]))
            else:
                self.position = self.position + self.speed*in_elapsed_time*((self.target-self.position)/np.linalg.norm(self.target-self.position))

        #if self.speed != 0.0:
            #print self.position
            #print self.target
            #print self.speed


    #render this leg in the 2D world.
    #uses 2 transformations to find leg position in world coordinates.
    #also returns the root position of the leg in world coords.
    #render step
    def renderLeg(self,in_mech_to_world,in_backing):
        outline_color = (255,0,0)
        leg_color = (0,255,0)
        tip_color = (0,255,0)

        transformMat = self.leg_to_mech*in_mech_to_world

        root_pos = np.matrix([0,0,1])*transformMat
        root_pos = (int(self.pixels_per_u*root_pos.item(0)),int(self.pixels_per_u*root_pos.item(1)))

        l1_pos = self.l1Pt*transformMat
        l1_pos = (int(self.pixels_per_u*l1_pos.item(0)),int(self.pixels_per_u*l1_pos.item(1)))
        l2_pos = self.l2Pt*transformMat
        l2_pos = (int(self.pixels_per_u*l2_pos.item(0)),int(self.pixels_per_u*l2_pos.item(1)))

        leg_pos = self.position*transformMat
        leg_pos = (int(self.pixels_per_u*leg_pos.item(0)),int(self.pixels_per_u*leg_pos.item(1)))
        targ_pos = self.target*transformMat
        targ_pos = (int(self.pixels_per_u*targ_pos.item(0)),int(self.pixels_per_u*targ_pos.item(1)))

        pygame.draw.line(in_backing,outline_color,root_pos,l1_pos)
        pygame.draw.line(in_backing,outline_color,root_pos,l2_pos)
        pygame.draw.line(in_backing,leg_color,root_pos,leg_pos)
        pygame.draw.circle(in_backing,tip_color,leg_pos,4)

        #draw our target positions.
        pygame.draw.circle(in_backing,(0,0,255),targ_pos,4)

        return root_pos, self.target*transformMat

