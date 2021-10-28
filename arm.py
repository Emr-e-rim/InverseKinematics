import numpy as np
import math
import pyglet
from numpy.core.numeric import array_equal
from numpy.core.records import array
import scipy.optimize  

class arm_segment:
    # TODO: Note to self choose array orientation wisely
    def __init__(self,len) -> None:
        self.length = len
        self.angle = math.pi / 4 # test 45 degree angle
        self.mount_position = np.array([0.0, 0.0, 0.0])
        self.mount_angle = 0.0
        self.end_position = np.array([0.0, 0.0, 0.0])

    def print(self):
        print("Arm length {0} angle {1} mount {2} endpoint {3} ".format(self.length,self.angle,self.mount_position, self.end_position))    

    def coords(self):
        print()

    def set_mount_position(self,pos,angle):
        self.mount_position = pos
        self.mount_angle = angle

    def calc_endpoint(self):
        # Beware all in radians.
        self.end_position[0] = self.mount_position[0] + self.length * math.sin(self.angle + self.mount_angle) #x-coord
        self.end_position[1] = self.mount_position[1] + self.length * math.cos(self.angle + self.mount_angle) #y-coord
    
    # hmmm OO needs a lot of boilerplatting.
    def get_endpoint(self):
        return self.end_position
    def get_combined_angle(self):
        return self.mount_angle + self.angle

    def rotate_to(self,target):
        def distance_to_default(angle, *args): 
            # weights found with trial and error, get some wrist bend, but not much
            weight = [1, 1, 1.3] 
            #print("distance:",np.sqrt(np.sum([(qi - q0i)**2 * wi
            #    for qi,q0i,wi in zip(angle, self.mount_position, weight)])))
            return np.sqrt(np.sum([(qi - q0i)**2 * wi
                for qi,q0i,wi in zip(angle, self.mount_position, weight)]))

        def x_constraint(angle, xy):
            x = (self.length*np.cos(angle)) - xy[0]
            #print("x:", x)
            return x

        def y_constraint(angle, xy): 
            y = (self.length*np.sin(angle)) - xy[1]
            #print("y:", y)
            return y
        
        #print("optimize",scipy.optimize.fmin_slsqp(
        #    func=distance_to_default,
        #    x0=self.angle,
        #    eqcons=[x_constraint,
        #            y_constraint],
            # uncomment to add in min / max angles for the joints
            # ieqcons=[joint_limits_upper_constraint,
            #          joint_limits_lower_constraint],
        #    args=(target,),
        #    iprint=0))  # iprint=0 suppresses output                print("Arm length {0} angle {1} mount {2} endpoint {3} ".format(self.length,self.angle,self.mount_position, self.end_position))
        #print("Arm length {0} angle {1} mount {2} endpoint {3} ".format(self.length,self.angle,self.mount_position, self.end_position))

        
        return scipy.optimize.fmin_slsqp(
            func=distance_to_default,
            x0=self.angle,
            eqcons=[x_constraint,
                    y_constraint],
            # uncomment to add in min / max angles for the joints
            # ieqcons=[joint_limits_upper_constraint,
            #          joint_limits_lower_constraint],
            args=(target,),
            iprint=0)  # iprint=0 suppresses output

        #pass
        # TODO:
        # step 1: move vectors to origin
        #target_vector = np.array([0.0,0.0,0.0])
        #target_vector[0] = target[0] - self.mount_position[0]
        #target_vector[1] = target[1] - self.mount_position[1]
        #target_vector[2] = target[2] - self.mount_position[2]

        #current_vector = np.array([0.0,0.0,0.0])
        #current_vector[0] = current[0] - self.mount_position[0]
        #current_vector[1] = current[1] - self.mount_position[1]
        #current_vector[2] = current[2] - self.mount_position[2]

        # step 2: calculate nomalized dot
        #normalized_target = target_vector/np.linalg.norm(target_vector)
        #normalized_current = current_vector/np.linalg.norm(current_vector)
        #dot = np.dot(normalized_target,normalized_current)

        # step 3: calc cross for direction
        #cross = np.cross(normalized_target,normalized_current)

        # update angle based on dot watch constraints
        #self.angle = dot



class robot_arm:
    def __init__(self) -> None:
        # TODO: Note to self choose array orientation wisely
        # third dimension for cross product.
        self.end_position = np.array([0.0, 0.0, 0.0])
        self.segments = []
        self.Xcoords = np.array([0.0])
        self.Ycoords = np.array([0.0])
    
    def add_segment(self,arm_segment):
        self.segments.append(arm_segment)

    def calc_endpoint(self):
        self.end_position = np.array([0.0,0.0,0.0]) # Initial mountpoint
        self.end_angle = 0.0 # initial angle in radians
        for s in self.segments:
            s.set_mount_position(self.end_position, self.end_angle)
            s.calc_endpoint()
            self.end_position = s.get_endpoint()
            self.end_angle = s.get_combined_angle()
            self.Xcoords = np.append(self.Xcoords, self.end_position[0])
            self.Ycoords = np.append(self.Ycoords, self.end_position[1])
    
    def move_to(self,pos):
        maximum_loopcount = 20 
        while  np.array_equal(pos,self.end_position) == False:
            #print(1)
            for seg in reversed(self.segments):
                #print(2)
                seg.rotate_to(pos)
           # print(3)
            maximum_loopcount -= 1
            #print(4)
            if maximum_loopcount == 0: break  
            #print(5)

    def print(self):
        self.calc_endpoint()
        for s in self.segments:
            s.coords()
            s.print()
        print('End angle: {0}'.format(self.end_angle))
        
arm = robot_arm()

arm.add_segment(arm_segment(100))
arm.add_segment(arm_segment(150))
arm.add_segment(arm_segment(80))

arm.print()
#print(arm.end_position[0], arm.end_position[1])
print(arm.move_to([200.0, 100.0]))

#arm.print()
"""A function for plotting an arm, and having it calculate the 
inverse kinematics such that given the mouse (x, y) position it 
finds the appropriate joint angles to reach that point."""

# make our window for drawin'
window = pyglet.window.Window()

label = pyglet.text.Label('Mouse (x,y)', font_name='Times New Roman', 
    font_size=36, x=window.width//2, y=window.height//2,
    anchor_x='center', anchor_y='center')

def get_joint_positions():
    """This method finds the (x,y) coordinates of each joint"""

    x = np.array([ 0, 
        arm.L[0]*np.cos(arm.q[0]),
        arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]),
        arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]) +
            arm.L[2]*np.cos(np.sum(arm.q)) ]) + window.width/2

    y = np.array([ 0, 
        arm.L[0]*np.sin(arm.q[0]),
        arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]),
        arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]) +
            arm.L[2]*np.sin(np.sum(arm.q)) ])

    return np.array([x, y]).astype('int')
    
#window.jps = get_joint_positions()
#window.jps = np.array([0+ window.width/2, 0, arm.end_position[0]+ window.width/2, arm.end_position[1]]).astype('int')
#arm.Xcoords = arm.Xcoords + window.width/2
#window.jps = np.array([arm.Xcoords, arm.Ycoords]).astype('int')
#print(window.jps)
#print(len(arm.Xcoords))

@window.event
def on_draw():
    arm.Xcoords = arm.Xcoords + window.width/2
    window.jps = np.array([arm.Xcoords, arm.Ycoords]).astype('int')
    window.clear()
    label.draw()
    for i in range(len(arm.Xcoords)-1): 
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', 
            (window.jps[0][i], window.jps[1][i], 
                window.jps[0][i+1], window.jps[1][i+1])
            ))

@window.event
def on_mouse_motion(x, y, dx, dy):
    # call the inverse kinematics function of the arm
    # to find the joint angles optimal for pointing at 
    # this position of the mouse 
    label.text = '(x,y) = (%.3f, %.3f)'%(x,y)
    #print("Coords:",x,y)
    #pos = [x,y]
    #arm.move_to(pos)
    #arm.q = arm.inv_kin([x - window.width/2, y]) # get new arm angles
    xy = np.array([x - window.width/2, y, 0.0])
    #print(xy)

    #arm.move_to(xy) # get new joint (x,y) positions

pyglet.app.run()
