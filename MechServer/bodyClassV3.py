#! /usr/bin/python

#Jeremy Lim
#This class represents the robot's chassis

#custom imports
from legClassV3 import MechLeg

#for rendering
#import pygame
#for matrix math
import numpy as np
import math

class MechBody:


    def __init__(self,in_pos,in_orientation,in_leg_sweep,in_leg_extend,in_leg_square_offset,in_leg_max,in_rads_max,in_dilate):
        #position, in world coords.
        self.position = in_pos
        #angle, with respect to x axis
        self.angleRads = 0.0#math.pi/4.0#in_orientation
        self.angleVelocity = 0
        self.world_mech_translate = np.transpose(np.matrix([[1,0,-self.position.item(0)],[0,1,-self.position.item(1)],[0,0,1]]))
        self.world_mech_rotate = np.transpose(np.matrix([[math.cos(self.angleRads),-math.sin(self.angleRads),0],[math.sin(self.angleRads),math.cos(self.angleRads),0],[0,0,1]]))
        #np.transpose(np.mat([[1,0,0],[0,-1,0],[0,0,1]]))*
        self.mech_to_world = np.linalg.inv(self.world_mech_rotate)*np.linalg.inv(self.world_mech_translate)
        #self.renderInverse = np.transpose(np.mat([[1,0,0],[0,-1,500],[0,0,1]]))
        #self.world_mech_rotate = np.transpose(np.matrix([[1,0,0],[0,-1,0],[0,0,1]]))
        in_pixels_unit = 1.0
        scaleFactor = in_leg_square_offset#1.0

        self.triDilate = in_dilate

        #stability leeway, in cm, of the center.
        self.center_stable_dist = 0.0#1.5
        self.center_stable_rads = 0.0#0.1

        self.lockedTime = 0
        self.prevLeg = -1
        legAngle = in_leg_sweep/180.0*math.pi#math.pi/2.0
        self.maxRadSpeed = in_rads_max#math.pi/4.0

        #variable that measures how long we've sat idle
        self.idle_time = 0
        #how long to wait(seconds) idle before the legs are repositioned.
        self.idle_time_limit = 2.5

        #initialize our list of legs
        #defaultStand = np.matrix([0,scaleFactor,1])
        defaultStand = np.matrix([0,in_leg_extend*0.6,1])
        #pi/4
        #l1 is FR
        l1Translate = np.transpose(np.matrix([[1,0,-scaleFactor],[0,1,-scaleFactor],[0,0,1]]))
        l1Rotate = np.matrix([[math.cos(-math.pi/4.0),-math.sin(-math.pi/4.0),0],[math.sin(-math.pi/4.0),math.cos(-math.pi/4.0),0],[0,0,1]])
        leg1 = MechLeg(in_pixels_unit,in_leg_extend,legAngle,l1Translate,l1Rotate,defaultStand)
        #list order: l1, l3, l2, l4

        #-pi/4
        #l2 is BR
        l2Translate = np.transpose(np.matrix([[1,0,-scaleFactor],[0,1,scaleFactor],[0,0,1]]))
        l2Rotate = np.matrix([[math.cos(-3.0*math.pi/4.0),-math.sin(-3.0*math.pi/4.0),0],[math.sin(-3.0*math.pi/4.0),math.cos(-3.0*math.pi/4.0),0],[0,0,1]])
        leg2 = MechLeg(in_pixels_unit,in_leg_extend,legAngle,l2Translate,l2Rotate,defaultStand)
        #3pi/4
        #l3 is FL
        l3Translate = np.transpose(np.matrix([[1,0,scaleFactor],[0,1,scaleFactor],[0,0,1]]))
        l3Rotate = np.matrix([[math.cos(3.0*math.pi/4.0),-math.sin(3.0*math.pi/4.0),0],[math.sin(3.0*math.pi/4.0),math.cos(3.0*math.pi/4.0),0],[0,0,1]])
        leg3 = MechLeg(in_pixels_unit,in_leg_extend,legAngle,l3Translate,l3Rotate,defaultStand)
        #-3pi/4
        #l4 is BL
        l4Translate = np.transpose(np.matrix([[1,0,scaleFactor],[0,1,-scaleFactor],[0,0,1]]))
        l4Rotate = np.matrix([[math.cos(math.pi/4.0),-math.sin(math.pi/4.0),0],[math.sin(math.pi/4.0),math.cos(math.pi/4.0),0],[0,0,1]])
        leg4 = MechLeg(in_pixels_unit,in_leg_extend,legAngle,l4Translate,l4Rotate,defaultStand)

        #create the list of legs
        self.legList = [leg1,leg2,leg3,leg4]
        #self.legList = [leg2]
        self.lifted_leg_index = -1

        self.gait_index = 0
        self.gait_type_index = 0
        #skip to this leg if this isn't -1.
        self.skip_index = -1
        self.dirNum = -1
        #self.skipLegList = [False,False,False,False]
        #top
        #[0,2,3,1]
        #left
        #[0,3,1,2]
        #bottom
        #[0,1,3,2]
        #right
        #[0,2,1,3]
        #use getDirectionNumber to index into this.
        #angular rotation, +theta, index 4
        #[0,1,2,3]
        #angular rotation, -theta, index 5
        #[0,3,2,1]
        self.gaitLists = [[0,2,1,3],[0,2,3,1],[0,3,1,2],[0,1,3,2],[0,1,2,3],[0,3,2,1]]

        #20 units per second.
        self.maxLegMove =  in_leg_max#80.0
        self.maxSlowMove = self.maxLegMove/3.0
        #actual velocity command
        self.velocityMove = np.matrix([0,0,0])

        self.turnPivot = np.matrix([0,0,1])

        self.prevVec = np.matrix([0,0,0])
        self.prev_Rot_vel = 0.0


    def constrainAngle(self,daAngle):
        if(daAngle > 2.0*math.pi):
            return daAngle - math.floor(daAngle/(2.0*math.pi))*2.0*math.pi
        elif(daAngle < 0.0):
            return 2.0*math.pi + (daAngle - math.ceil(daAngle/(2.0*math.pi))*2.0*math.pi)
        else:
            return daAngle

    def getDirectionNumber(self,in_ang):
        #angle from 0 to 2pi
        if(in_ang > 7.0*math.pi/4.0 or in_ang < math.pi/4.0):
            return 0
        elif(in_ang >= math.pi/4.0 and in_ang < 3.0*math.pi/4.0):
            return 1
        elif(in_ang >= 3*math.pi/4.0 and in_ang < 5.0*math.pi/4.0):
            return 2
        else:
            return 3
            
    def getBestOppositeLeg(self,in_angle):
        #angle from 0 to 2pi
        if(in_angle > 0.0 and in_angle < math.pi/2.0):
            return 2
        if(in_angle >= math.pi/2.0 and in_angle < math.pi):
            return 1
        if(in_angle >= math.pi and in_angle < 3.0*math.pi/2.0):
            return 0
        else:
            return 3

    #precondition: all arguments are in homogenous coordinates.
    def getLineIntersectDist(self,start_pt,in_direction,in_pt_a,in_pt_b):
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

    #precondition: all arguments are in homogenous coordinates.
    def getSignedLineIntersectDist(self,start_pt,in_direction,in_pt_a,in_pt_b):
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
            intersectPt = (tParam*in_direction + start_pt) - in_pt_a
            v_param = in_pt_b - in_pt_a

            t2 = intersectPt.item(0)/v_param.item(0)
            t3 = intersectPt.item(1)/v_param.item(1)

            #no intersection "behind" the vector...
            #start from point a.
            #if t2 != t3:

            is_parallel = (not t2 >= 0)
            #if is_parallel:

        return dist, is_parallel

    #the third point is assumed to be 0
    def checkInsideArea(self,check_pos,vecA,vecB):
        check_vec = check_pos
        check_vec[0,2] = 0
        c_length = np.linalg.norm(check_vec)
        a_length = np.linalg.norm(vecA)
        b_length = np.linalg.norm(vecB)

        CA = np.vdot(np.asarray(check_vec),np.asarray(vecA))/(c_length*a_length)
        if CA > 1.0:
            CA = 1.0
        if CA < -1.0:
            CA = -1.0
        CB = np.vdot(np.asarray(check_vec),np.asarray(vecB))/(c_length*b_length)
        if CB > 1.0:
            CB = 1.0
        if CB < -1.0:
            CB = -1.0
        BA =  np.vdot(np.asarray(vecB),np.asarray(vecA))/(b_length*a_length)
        if BA > 1.0:
            BA = 1.0
        if BA < -1.0:
            BA = -1.0
        thetaCA = math.acos(CA)
        thetaCB = math.acos(CB)
        thetaBA = math.acos(BA)

        #do the angles sum properly?
        deltaTheta = abs(thetaBA-(thetaCA+thetaCB))
        #true if we're in the area...
        if deltaTheta < 0.001:
            return True
        else:
            return False
        #return (deltaTheta < 0.001)


    #how far to move the picked up leg
    #to guarantee we won't fall over 0-2 steps into the future.
    def findLegTraversal(self,in_dir_vec,in_rot_vel,in_pivot_pt,first_dist_constrain,speed):
        #forward distance
        if in_rot_vel == 0.0:
            maximum, nopetarg, nopeDist = self.legList[self.lifted_leg_index].extensionDist(in_dir_vec,False,True,self.legList[self.lifted_leg_index].getStandPos())
            maximum = maximum - 0.5
            if maximum < 0:
                maximum = 0.0
            #backward distance
            minimum, nopetarg, nopeDist = self.legList[self.lifted_leg_index].extensionDist(in_dir_vec,True,True,self.legList[self.lifted_leg_index].getStandPos())
            minimum = minimum - 0.5
            if minimum < 0:
                minimum = 0.0
        else:
            #nonzero rotational velocity
            maximum, nopetarg, nopeDist = self.legList[self.lifted_leg_index].circularExtensionDist(in_dir_vec,in_pivot_pt,False,True,self.legList[self.lifted_leg_index].getStandPos(),(in_rot_vel>0.0))
            maximum = maximum - 0.001
            if maximum < 0:
                maximum = 0.0
            #backward distance
            minimum, nopetarg, nopeDist = self.legList[self.lifted_leg_index].circularExtensionDist(in_dir_vec,in_pivot_pt,True,True,self.legList[self.lifted_leg_index].getStandPos(),(in_rot_vel<0.0))
            minimum = minimum - 0.001
            if minimum < 0:
                minimum = 0.0

        #make it signed.
        minimum = -minimum
        unconstrainedLength = maximum-minimum
        unconstrainedMax = maximum
        
        #print "min: " + str(minimum) + "max: " + str(maximum)

        #find the new mins & maxes allowed by the next-created stability triangle.
        #leg that is currently lifted
        leg_a = self.legList[self.lifted_leg_index]
        #next one
        leg_c = self.legList[self.gaitLists[self.gait_type_index][self.gait_index]]
        next_ind = self.gait_index + 1
        if next_ind > 3:
            next_ind = 0
        #one after that
        leg_d = self.legList[self.gaitLists[self.gait_type_index][next_ind]]
        next_ind += 1
        if next_ind > 3:
            next_ind = 0
        #this one never moves...
        leg_b = self.legList[self.gaitLists[self.gait_type_index][next_ind]]

        if in_rot_vel == 0.0: 
            b_step_1 = leg_b.legTipPosition() + -first_dist_constrain*in_dir_vec/np.linalg.norm(in_dir_vec)
            d_step_1 = leg_d.legTipPosition() + -first_dist_constrain*in_dir_vec/np.linalg.norm(in_dir_vec)
            #c_step_1 = leg_c.legTipPosition() + -first_dist_constrain*in_dir_vec/np.linalg.norm(in_dir_vec)
        else:
            #in this case, first dist is in radians.
            if in_rot_vel > 0.0:#counterclockwise
                modifiedConstrain = -first_dist_constrain
            else:
                modifiedConstrain = first_dist_constrain

            bVec = leg_b.legTipPosition()-in_pivot_pt
            b_step_1 = in_pivot_pt + bVec*np.transpose(np.matrix([[math.cos(modifiedConstrain),-math.sin(modifiedConstrain),0],[math.sin(modifiedConstrain),math.cos(modifiedConstrain),0],[0,0,1]]))

            dVec = leg_d.legTipPosition()-in_pivot_pt
            d_step_1 = in_pivot_pt + dVec*np.transpose(np.matrix([[math.cos(modifiedConstrain),-math.sin(modifiedConstrain),0],[math.sin(modifiedConstrain),math.cos(modifiedConstrain),0],[0,0,1]]))


        #print b_step_1
        #print d_step_1

        newMin = minimum
        newMax = maximum

        frontList = []
        backList = []

        if in_rot_vel == 0.0: 
            intersectA, parallel = self.getSignedLineIntersectDist(leg_a.getStandPosMech(),in_dir_vec,np.matrix([0,0,1]),-1*b_step_1)
            if not parallel and intersectA >= 0.0:
                frontList.append(intersectA)
            elif not parallel:
                backList.append(intersectA)
        else:
            front = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*b_step_1*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel < 0.0))
            back = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*b_step_1*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel > 0.0))
            frontList = frontList + front
            for d in range(0,len(back)):
                back[d] = -back[d]
            backList = backList + back

        if in_rot_vel == 0.0:
            intersectB, parallel = self.getSignedLineIntersectDist(leg_a.getStandPosMech(),in_dir_vec,np.matrix([0,0,1]),-1*d_step_1)
            if not parallel and intersectB >= 0.0:
                frontList.append(intersectB)
            elif not parallel:
                backList.append(intersectB)
        else:
            front = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*d_step_1*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel < 0.0))
            back = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*d_step_1*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel > 0.0))
            frontList = frontList + front
            for d in range(0,len(back)):
                back[d] = -back[d]
            backList = backList + back

        #print frontList
        #print backList
        #are we in the correct area already? or not?
        if self.checkInsideArea(leg_a.getStandPosMech(),np.matrix([0,0,1])-b_step_1,np.matrix([0,0,1])-d_step_1):#inside
            if len(frontList) != 0:
                newMax = min(frontList)
            if len(backList) != 0:
                newMin = max(backList)
        else:#outside
            combList = frontList + backList
            if len(combList) != 0:
                newMax = max(combList)
                newMin = min(combList)

        #NOTE:UNCOMMENT
        if newMin > maximum:
            #print "ain't that a kick in the head..."
            #constrain as far as possible.
            minimum = maximum
        elif newMax < minimum:
            #print "ain't that a kick in the head..."
            maximum = minimum
        else:
            maximum = min(maximum,newMax)
            minimum = max(minimum,newMin)


        #print "stage 2:"
        #print "min: " + str(minimum) + "max: " + str(maximum)
        #now, find the stability triangle 2 steps in the future.
        #2 cases: Lifted leg determines translation distance, or not.
        #in a perfect world, leg_a does not determine the minimum, nor does the stability triangle.

        #in the perfect case, it's the min of B+1's and D+1's reverse extension
        #d_step_2_start
        #b_step_2_start = 
        if in_rot_vel == 0.0:
            bdist, nopetarg, nopeDist = leg_b.extensionDist(in_dir_vec,True,True,leg_b.goMechtoLeg(b_step_1))
            ddist, nopetarg, nopeDist = leg_d.extensionDist(in_dir_vec,True,True,leg_d.goMechtoLeg(d_step_1))
        else:
            bdist, nopetarg, nopeDist = leg_b.circularExtensionDist(in_dir_vec,in_pivot_pt,True,True,leg_b.goMechtoLeg(b_step_1),(in_rot_vel<0.0))
            ddist, nopetarg, nopeDist = leg_d.circularExtensionDist(in_dir_vec,in_pivot_pt,True,True,leg_d.goMechtoLeg(d_step_1),(in_rot_vel<0.0))
        #cdist, nopetarg, nopeDist = leg_c.extensionDist(in_dir_vec,True,True,leg_c.goMechtoLeg(c_step_1))
        b2dist = min(bdist,ddist)
        #case 2: matter of the lifted leg.

        if in_rot_vel == 0.0:
            b_step_2 = b_step_1 + -b2dist*in_dir_vec/np.linalg.norm(in_dir_vec)
            c_extend, nopetarg, nopeDist = leg_c.extensionDist(in_dir_vec,False,True,leg_c.getStandPos())
            c_step_2 =  leg_c.getStandPosMech() + c_extend*in_dir_vec/np.linalg.norm(in_dir_vec)
        else:
            if in_rot_vel > 0.0:#counterclockwise
                modifiedConstrain = -b2dist
            else:
                modifiedConstrain = b2dist
            bVec = b_step_1-in_pivot_pt
            b_step_2 = in_pivot_pt + bVec*np.transpose(np.matrix([[math.cos(modifiedConstrain),-math.sin(modifiedConstrain),0],[math.sin(modifiedConstrain),math.cos(modifiedConstrain),0],[0,0,1]]))

            c_extend, c_step_2, nopeDist = leg_c.circularExtensionDist(in_dir_vec,in_pivot_pt,False,True,leg_c.getStandPos(),(in_rot_vel>0.0))
            c_step_2 = leg_c.goLegtoMech(c_step_2)
            #cVec = 
            #c_step_2 = in_pivot_pt + bVec*np.transpose(np.matrix([[math.cos(modifiedConstrain),-math.sin(modifiedConstrain),0],[math.sin(modifiedConstrain),math.cos(modifiedConstrain),0],[0,0,1]]))
            #c_step_2 =  leg_c.getStandPosMech() + c_extend*in_dir_vec/np.linalg.norm(in_dir_vec)


        newMin = minimum
        newMax = maximum

        frontList = []
        backList = []

        if in_rot_vel == 0.0:
            intersectA, parallel = self.getSignedLineIntersectDist(leg_a.getStandPosMech(),in_dir_vec,np.matrix([0,0,1]),-1*c_step_2)
            if not parallel and intersectA >= 0.0:
                frontList.append(intersectA)
            elif not parallel:
                backList.append(intersectA)
        else:
            front = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*c_step_2*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel < 0.0))
            back = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*c_step_2*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel > 0.0))
            frontList = frontList + front
            for d in range(0,len(back)):
                back[d] = -back[d]
            backList = backList + back

        if in_rot_vel == 0.0:
            intersectB, parallel = self.getSignedLineIntersectDist(leg_a.getStandPosMech(),in_dir_vec,np.matrix([0,0,1]),-1*b_step_2)
            if not parallel and intersectB >= 0.0:
                frontList.append(intersectB)
            elif not parallel:
                backList.append(intersectB)
        else:
            front = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*b_step_2*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel < 0.0))
            back = self.radsParameterizedlineIntersect(np.matrix([0,0,1]),-1*b_step_2*10000,leg_a.getStandPosMech(),in_pivot_pt,(in_rot_vel > 0.0))
            frontList = frontList + front
            for d in range(0,len(back)):
                back[d] = -back[d]
            backList = backList + back

        #are we in the correct area already? or not?
        if self.checkInsideArea(leg_a.getStandPosMech(),np.matrix([0,0,1])-b_step_2,np.matrix([0,0,1])-c_step_2):#inside
            if len(frontList) != 0:
                newMax = min(frontList)
            if len(backList) != 0:
                newMin = max(backList)
        else:#outside
            combList = frontList + backList
            if len(combList) != 0:
                newMax = max(combList)
                newMin = min(combList)
            #else:

        #NOTE: UNCOMMENT
        if newMin > maximum:
            #print "ain't that a hole in your boat..."
            #constrain as far as possible.
            minimum = maximum
        elif newMax < minimum:
            #print "ain't that a hole in your boat..."
            maximum = minimum
        else:
            maximum = min(maximum,newMax)
            minimum = max(minimum,newMin)

        #print "stage 3:"
        #print "min: " + str(minimum) + " max: " + str(maximum)
        #attempt at helping convergence...
        #print unconstrainedLength

        #print (maximum-minimum)
        #print unconstrainedMax
        #print maximum
        if unconstrainedLength == (maximum-minimum):
            #print "maximal reduction"
            maximum = minimum+unconstrainedLength*0.667#0.667
        elif maximum == unconstrainedMax and maximum-minimum > 0.1:
            maximum = maximum - 0.1

        #Move inside our specified range.
        if in_rot_vel == 0.0:
            leg_target = leg_a.getStandPosMech() + maximum*in_dir_vec/np.linalg.norm(in_dir_vec)
        else:
            lVec = leg_a.getStandPosMech()-in_pivot_pt
            #legRad = np.linalg.norm(legRad)
            if in_rot_vel < 0.0:
                maximum = -abs(maximum)
            else:
                maximum = abs(maximum)
            leg_target = in_pivot_pt + lVec*np.transpose(np.matrix([[math.cos(maximum),-math.sin(maximum),0],[math.sin(maximum),math.cos(maximum),0],[0,0,1]]))

        liftDist = np.linalg.norm(leg_target - leg_a.legTipPosition())

        if in_rot_vel == 0.0:
            if first_dist_constrain != 0.0 and speed != 0.0:
                fastSpeed = liftDist/(first_dist_constrain/speed)
                if fastSpeed > self.maxLegMove:
                    fastSpeed = self.maxLegMove
            else:
                fastSpeed = self.maxLegMove
        else:
            if first_dist_constrain != 0.0:
                fastSpeed = abs(liftDist/(first_dist_constrain/abs(in_rot_vel)))
                #fastSpeed = fastRadSpeed*legRad
                if fastSpeed > self.maxLegMove:
                    fastSpeed = self.maxLegMove
            else:
                fastSpeed = self.maxLegMove

        #print "Leg's target: " + str(leg_target)
        leg_a.setTarget(leg_a.goMechtoLeg(leg_target),fastSpeed)


    def radsParameterizedlineIntersect(self,pt_a,pt_b,leg_pt,turn_center,clockwise):
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
            retList = clockwiseList
        else:
            retList = counterClockwiseList

        for g in range(0,len(retList)):
            retList[g] = retList[g]/radius2

        return retList

    #calculates the distance of the origin to the edge of the stabilty triangle
    #in a certain direction
    #precondition: in_pts_list has exactly 3 items.
    def stabilityTriangleDist(self,in_mech_velocity,in_pts_list,in_pivot_pt,in_mech_angular_vel):
        mechRadius = np.linalg.norm(in_pivot_pt)
        #are we going perfectly diagonal?
        if True or abs(in_mech_velocity.item(0)) == abs(in_mech_velocity.item(0)):
            #dilate our points slightly (by 1 unit)
            dilate_length = self.triDilate

            centroid = (in_pts_list[0] + in_pts_list[1] + in_pts_list[2])/3.0
            centroid[0,2] = 0.0
            #0-1 side vect
            side01 = (in_pts_list[0] + in_pts_list[1])/2.0 - centroid
            side01[0,2] = 0.0
            side01 = side01/np.linalg.norm(side01)
            #1-2 side vect
            side12 = (in_pts_list[1] + in_pts_list[2])/2.0 - centroid
            side12[0,2] = 0.0
            side12 = side12/np.linalg.norm(side12)
            #2-0 side vect
            side20 = (in_pts_list[2] + in_pts_list[0])/2.0 - centroid
            side20[0,2] = 0.0
            side20 = side20/np.linalg.norm(side20)
            #expanded list
            exp_list = []
            exp_list.append((in_pts_list[0] + dilate_length*side01 + dilate_length*side20))
            exp_list.append((in_pts_list[1] + dilate_length*side01 + dilate_length*side12))
            exp_list.append((in_pts_list[2] + dilate_length*side12 + dilate_length*side20))
        else:
            exp_list = in_pts_list


        plus_list = []
        #minus_list = []
        center = np.matrix([0,0,0])

        isClock = (in_mech_angular_vel < 0)

        if in_pivot_pt.item(2) == 0.0:
            calc_dist,parallel = self.getLineIntersectDist(center,in_mech_velocity,exp_list[0],exp_list[2])
        else:
            lst = self.radsParameterizedlineIntersect(exp_list[0],exp_list[2],center,in_pivot_pt,isClock)
            if len(lst) == 0:
                parallel = True
            else:
                parallel = False
                calc_dist = min(lst)

        if not parallel:
            if calc_dist >= 0:
                plus_list.append(calc_dist)
            #else:
                #minus_list.append(calc_dist)

        for b in range(1,3):
            if in_pivot_pt.item(2) == 0.0:
                calc_dist, parallel = self.getLineIntersectDist(center,in_mech_velocity,exp_list[b-1],exp_list[b])
            else:
                lst = self.radsParameterizedlineIntersect(exp_list[0],exp_list[2],center,in_pivot_pt,isClock)
                if len(lst) == 0:
                    parallel = True
                else:
                    parallel = False
                    calc_dist = min(lst)

            if not parallel:
                if calc_dist >= 0:
                    plus_list.append(calc_dist)
                #else:
                    #minus_list.append(calc_dist)
        if len(plus_list) != 0:
            plus_list.sort()
            if len(plus_list) >= 2:
                if plus_list[0] == 0.0 or True:
                    return plus_list[1]
                else:
                    return plus_list[0] 
            else:
                return plus_list[0]
        #elif len(minus_list) != 0:
            #return 0.0#max(minus_list)
        else:
            return 0.0



    def setCommand(self,in_world_velocity,in_rotation_vel):
        #find if the direction integer has changed
        vec = in_world_velocity#*self.world_mech_rotate
        if abs(in_rotation_vel) < 0.000000001:
            in_rotation_vel = 0.0

        if(vec.item(0) == 0.0 and vec.item(1) == 0.0):
            if in_rotation_vel == 0.0:
                #if self.idle_time != 0
                #self.idle_time = k
                if self.idle_time < self.idle_time_limit:
                    for thing in self.legList:
                        thing.stahp()
                        self.velocityMove = np.matrix([0,0,0])
                        #not sure
                        #self.dirNum = -1
                    self.angleVelocity = 0
                return

        if np.array_equal(vec,self.prevVec) and in_rotation_vel == self.prev_Rot_vel and self.lifted_leg_index != -1:
            return

        #Will only do extensive calculations if absolutely required.
        self.prevVec = vec
        self.prev_Rot_vel = in_rotation_vel
        #decide on leg gait order if:
        #direction changes or
        #an initial direction is chosen.
        if in_rotation_vel != 0 and vec.item(0) == 0.0 and vec.item(1) == 0.0:
            #choose arbitrary direction?
            #if self.dirNum == -1:
                #newNum = 0
            #else:
                #newNum = self.dirNum
            if in_rotation_vel > 0.0:
                newNum = 5
            else:
                newNum = 4
        else:
            newNum = self.getDirectionNumber(self.constrainAngle(math.atan2(vec.item(1),vec.item(0))))

        #decide gait type
        if self.gait_type_index  == -1 or newNum != self.dirNum:
            self.gait_type_index = newNum
            self.dirNum = newNum
            #replaint the index, if possible
            if self.lifted_leg_index != -1:
                self.gait_index = 0
                while self.gaitLists[self.gait_type_index][self.gait_index] != self.lifted_leg_index:
                    self.gait_index += 1
                    if self.gait_index > 3:
                        self.gait_index = 0
                
            else:
                legInd = self.getBestOppositeLeg(self.constrainAngle(math.atan2(vec.item(1),vec.item(0))))
                self.gait_index = 0
                while self.gaitLists[self.gait_type_index][self.gait_index] != legInd:
                    self.gait_index += 1
                    if self.gait_index > 3:
                        self.gait_index = 0

        #decide leg to lift (if applicable)
        #all legs are on the ground
        if self.lifted_leg_index == -1:
            self.lifted_leg_index = self.gaitLists[self.gait_type_index][self.gait_index]
            self.legList[self.lifted_leg_index].setFootState(False)
            self.gait_index += 1
            if self.gait_index > 3:
                self.gait_index = 0

        #find the max distance the body and 3 non-standing legs
        distList = []
        triangleList = []
        #max distance of the stability triangle.

        #testing!
        #forwardDists = []

        #calculate our turning point, if there is one.
        
        if in_rotation_vel != 0:
            turnRadius = np.linalg.norm(vec)/in_rotation_vel
            #print "radius" + str(turnRadius)
            #print turnRadius
            #90 deg. rotation
            rot = math.pi/2.0
            if np.linalg.norm(vec) != 0.0:
                turnPosMech = (vec/np.linalg.norm(vec))*turnRadius*np.transpose(np.matrix([[math.cos(rot),-math.sin(rot),0],[math.sin(rot),math.cos(rot),0],[0,0,1]]))
                turnPosMech[0,2] = 1.0
            else:
                turnPosMech = np.mat([0,0,1])

        else:
            #turnRadius = 
            turnPosMech = np.mat([0,0,0])


        for a in range(0,4):
            if self.lifted_leg_index != a:
                if in_rotation_vel != 0.0:
                    calcdist, calcposition, waste = (self.legList[a]).circularExtensionDist(vec,turnPosMech,True,False,(0,0,0),(in_rotation_vel<0.0))
                    calcdist -= 0.01
                    if calcdist < 0.0:
                        calcdist = 0.0
                else:
                    calcdist, calcposition, waste = (self.legList[a]).extensionDist(vec,True,False,(0,0,0))
                    calcdist -= 0.5
                    if calcdist < 0.0:
                        calcdist = 0.0

                distList.append(calcdist)
                triangleList.append(self.legList[a].legTipPosition())

        if len(triangleList) == 3:
            if vec.item(0) != 0.0 or vec.item(1) != 0.0:
                if in_rotation_vel != 0.0:
                    distValue = self.stabilityTriangleDist(vec,triangleList,turnPosMech,in_rotation_vel)-self.center_stable_rads
                    if distValue < 0.0:
                        distValue = 0.0
                    distList.append(distValue)
                else:
                    distValue = self.stabilityTriangleDist(vec,triangleList,turnPosMech,in_rotation_vel)-self.center_stable_dist
                    if distValue < 0.0:
                        distValue = 0.0
                    distList.append(distValue)

        #print distList
        maximalShift = min(distList)
        #try to prevent leg lock-up
        if in_rotation_vel != 0.0:
            lockNum = self.determineLockedLegs(distList)
            if lockNum == 1:
                maximalShift = maximalShift - 0.05
                #print "antilock1"
            elif lockNum == 2:
                maximalShift = maximalShift - 0.1
                #print "antilock2"
            elif lockNum >= 3:
                maximalShift = maximalShift - 0.15
                #print "antilock3"
            #else:
                #print "no lock"

        #print distList
        #if in_rotation_vel != 0.0:
            #maximalShift = 0.5*maximalShift

        if maximalShift < 0.0:
            maximalShift = 0.0

        if abs(maximalShift) < 0.0001:
            maximalShift = 0.0

        #print maximalShift
        #now we can calculate the leg commands.

        if in_rotation_vel != 0:
            #can't solve for a single speed.
            slowRadSpeed = in_rotation_vel
            if abs(in_rotation_vel) > self.maxRadSpeed:
                if in_rotation_vel < 0.0:
                    slowRadSpeed = -self.maxRadSpeed
                else:
                    slowRadSpeed = self.maxRadSpeed
        else:
            if np.linalg.norm(vec) > self.maxSlowMove:
                slow_speed = self.maxSlowMove
            else:
                slow_speed = np.linalg.norm(vec)
            set_position = (vec/np.linalg.norm(vec))*maximalShift*-1

        for c in range(0,4):
            if self.lifted_leg_index != c:
                if in_rotation_vel != 0:
                    if in_rotation_vel < 0:
                        self.legList[c].setTargetAngleExtend(maximalShift,slowRadSpeed,turnPosMech)
                    else:
                        self.legList[c].setTargetAngleExtend(-maximalShift,slowRadSpeed,turnPosMech)
                        #self.legList[c].setTargetAngleExtend(u5.000001,slowRadSpeed,turnPosMech)
                else:
                    self.legList[c].setTargetExtend(set_position,slow_speed)
        #the mech body's movement must be set.
        if in_rotation_vel != 0:
            #relative to the world
            self.turnPivot = turnPosMech
            if maximalShift > 0.0:
                self.angleVelocity = slowRadSpeed
                self.velocityMove = turnRadius*self.angleVelocity
            else:
                self.angleVelocity = 0.0
                self.velocityMove = np.matrix([0,0,0])
        else:
            self.angleVelocity = 0.0
            if maximalShift > 0.0:
                self.velocityMove = slow_speed*(in_world_velocity/np.linalg.norm(in_world_velocity))*self.mech_to_world
            else:
                self.velocityMove = np.matrix([0,0,0])


        if self.lifted_leg_index != -1:
            #new method for commanding the leg:
            if in_rotation_vel == 0.0:
                self.findLegTraversal(vec,0.0,turnPosMech,maximalShift,slow_speed)
            else:
                self.findLegTraversal(vec,slowRadSpeed,turnPosMech,maximalShift,0.0)

    def determineLockedLegs(self,in_distList):
        workList = []
        for r in range(0,len(in_distList)):
            #ignore zeros.
            #if abs(in_distList[r]) > 0.000001:
            workList.append(in_distList[r])

        #cross check how many matching terms we have
        matchNum = 0
        for a in range(0,len(workList)-1):
            for b in range(a+1,len(workList)):
                #floating-point close-enough comparizon
                if abs(workList[a]-workList[b]) < 0.000001:
                    matchNum += 1
            
        return matchNum

    def commandBackToStand(self):
        #first, halt the legs
        #print "leg reset mode."

        if self.lifted_leg_index == -1:
            potentialList = []
            maxDist = self.isStableConfig(0)
            if maxDist > 0:
                minLeg = 0
            else:
                minLeg = -1

            for b in range(1,4):
                value = self.isStableConfig(b)
                if value > 0 and value > maxDist:
                    minLeg = b
                    maxDist = value

            #if minLeg == -1:
                #print "stable stand"

            #we've chosen the leg to lift
            self.lifted_leg_index = minLeg

        #command our lifted leg to the standing position
        #we are guaranteed to have a leg to lift at this point
        if self.lifted_leg_index != -1:
            for c in range(0,4):
                if self.lifted_leg_index != c:
                    self.legList[c].stahp() 
                else:
                    self.legList[c].setTarget(self.legList[c].stand_pos,self.maxLegMove)#self.maxLegMove
                    self.legList[c].setFootState(False)
        else:
            for thing in self.legList:
                thing.stahp()


    #returns a positive number being the distance from the excluded
    #leg to the stand position, if it can lift it without falling
    #otherwise, return -1.
    def isStableConfig(self,excludeIndex):
        liftLeg = self.legList[excludeIndex]
        liftDist  = np.linalg.norm(liftLeg.getStandPos()-liftLeg.position)
        if liftDist > 0.0000001:
            in_pts_list = []
            
            for L in range(0,4):
                if L != excludeIndex:
                    in_pts_list.append(self.legList[L].legTipPosition())

            #dilate our points slightly (by 1 unit)
            dilate_length = 2.0

            centroid = (in_pts_list[0] + in_pts_list[1] + in_pts_list[2])/3.0
            centroid[0,2] = 0.0
            #0-1 side vect
            side01 = (in_pts_list[0] + in_pts_list[1])/2.0 - centroid
            side01[0,2] = 0.0
            side01 = side01/np.linalg.norm(side01)
            #1-2 side vect
            side12 = (in_pts_list[1] + in_pts_list[2])/2.0 - centroid
            side12[0,2] = 0.0
            side12 = side12/np.linalg.norm(side12)
            #2-0 side vect
            side20 = (in_pts_list[2] + in_pts_list[0])/2.0 - centroid
            side20[0,2] = 0.0
            side20 = side20/np.linalg.norm(side20)
            #expanded list
            exp_list = []
            exp_list.append((in_pts_list[0] + dilate_length*side01 + dilate_length*side20))
            exp_list.append((in_pts_list[1] + dilate_length*side01 + dilate_length*side12))
            exp_list.append((in_pts_list[2] + dilate_length*side12 + dilate_length*side20))

            plus_list = []
            center = np.matrix([0,0,0])
            testDir = np.matrix([0,1,0])

            calc_dist,parallel = self.getLineIntersectDist(center,testDir,exp_list[0],exp_list[2])

            if not parallel:
                if calc_dist >= 0:
                    plus_list.append(calc_dist)

            for b in range(1,3):
                calc_dist, parallel = self.getLineIntersectDist(center,testDir,exp_list[b-1],exp_list[b])

                if not parallel:
                    if calc_dist >= 0:
                        plus_list.append(calc_dist)

            #if we intersect an odd number of times, we're inside
            if len(plus_list) % 2 != 0.0:
                return liftDist
            else:
                return -1.0
        else:
            return 0.0


    def updateState(self,in_elapsed_time):

        if self.angleVelocity == 0.0 and np.linalg.norm(self.velocityMove) < 0.000000001:
            #print self.idle_time
            self.idle_time += in_elapsed_time
            if self.idle_time >= self.idle_time_limit:
                #command the legs to return to stand position.
                self.commandBackToStand()       
                self.idle_time = self.idle_time_limit
        else:
            self.idle_time = 0.0

        #update legs.
        for leg in self.legList:
            leg.updateState(in_elapsed_time)

        if self.lifted_leg_index != -1:
            if self.legList[self.lifted_leg_index].getFootState() == True:
                self.lifted_leg_index = -1
    
        #update body position
        if self.angleVelocity == 0.0:
            self.position = self.position+self.velocityMove*in_elapsed_time
        else:
            selfVec = self.position - self.turnPivot*self.mech_to_world
            rot = self.angleVelocity*in_elapsed_time
            self.position = self.turnPivot*self.mech_to_world + selfVec*np.transpose(np.matrix([[math.cos(rot),-math.sin(rot),0],[math.sin(rot),math.cos(rot),0],[0,0,1]]))
            self.angleRads -= self.angleVelocity*in_elapsed_time

        #update coordinate transformations.
        self.world_mech_translate = np.transpose(np.matrix([[1,0,-self.position.item(0)],[0,1,-self.position.item(1)],[0,0,1]]))
        self.world_mech_rotate = np.transpose(np.matrix([[math.cos(self.angleRads),-math.sin(self.angleRads),0],[math.sin(self.angleRads),math.cos(self.angleRads),0],[0,0,1]]))
        self.mech_to_world = np.linalg.inv(self.world_mech_rotate)*np.linalg.inv(self.world_mech_translate)#*np.transpose(np.mat([[1,0,0],[0,-1,500],[0,0,1]]))

    def get2DLegInfos(self):
        l1 = self.legList[0]
        l2 = self.legList[1]
        l3 = self.legList[2]
        l4 = self.legList[3]
        #list order: l1, l3, l2, l4
        #also, return the index of the lifted leg.
        retList = []
        distList = []
        distList.append(l1.getTargDist())
        retList.append(l1.legTipRelative())
        distList.append(l2.getTargDist())
        retList.append(l2.legTipRelative())
        distList.append(l3.getTargDist())
        retList.append(l3.legTipRelative())
        distList.append(l4.getTargDist())
        retList.append(l4.legTipRelative())

        liftIndex = self.lifted_leg_index
        #for r in range(0,4):
            #if not self.legList[r].getFootState():#it is grounded
                #liftIndex = r

        return retList, distList, liftIndex

#Render method removed.
