import numpy as np
import math
import matplotlib.pyplot as plt

# Set threshold and constraint
THRESHOLD = 0.01
CONSTRAINT = 5.0

# Create figure
fig = plt.figure() 
ax = fig.add_subplot(1,1,1)

def create_figure():
    plt.cla()
    # Add title tot figure
    fig.suptitle("Click to move arm", fontsize=18)
    # Define axis limits
    ax.set_xlim(-25, 50)
    ax.set_ylim(0, 50)

class arm_segment:
    def __init__(self,len, angle) -> None:
        """Set up the basic parameters of the arm segment.
        len : int
            the initial length of the arm segment
        angle : int
            the initial angle of the arm segment (in degrees)
        """
        self.length = len
        self.angle = math.radians(angle) # Convert to radians to use in program
        # Initial arm segment mount/end coordinations and angle
        self.mount_position = np.array([0.0, 0.0, 0.0])
        self.mount_angle = 0.0
        self.end_position = np.array([0.0, 0.0, 0.0])

    def print(self):
        """Returns the data of the arm segment."""
        print("Arm length {0}, angle {1}, mount x: {2} y: {3}, endpoint x: {4} y: {5}".format(
            self.length,math.degrees(self.angle),self.mount_position[0], self.mount_position[1],
            self.end_position[0], self.end_position[1]))    

    def plot(self):
        """Plots the arm segment in the figure"""
        ax.plot([self.mount_position[0], self.end_position[0]], [self.mount_position[1], self.end_position[1]], linewidth=3)
        fig.canvas.draw()

    def set_mount_position(self,pos,angle):
        """Updates the arm segment data.
        pos : np.array
            the end coordinations of the previous arm segment, the next segment mounts on this position
        angle : float
            the end angle of the previous arm segment
        """
        self.mount_position = pos
        self.mount_angle = angle

    def calc_endpoint(self):
        """Calculates the end coordinations of the arm segment"""
        self.end_position[0] = self.mount_position[0] + self.length * math.sin(self.angle + self.mount_angle)
        self.end_position[1] = self.mount_position[1] + self.length * math.cos(self.angle + self.mount_angle)
    
    def get_endpoint(self):
        """Returns the end position of the arm segment"""
        return self.end_position

    def get_combined_angle(self):
        """Returns the end angle"""
        return self.mount_angle + self.angle

    def rotate_to(self,target,current):
        """Rotates the arm to the target position using inverse kinematics
        target : np.array
            x, y and z coords of the target position
        current : np.array
            x, y and z coords of the current position
        """
        # Get root and end coords of the arm segment, separately
        root_posX = self.mount_position[0]
        root_posY = self.mount_position[1]
        root_posZ = self.mount_position[2]

        current_endX = current[0]
        current_endY = current[1]
        current_endZ = current[2]
        
        # Get target coords, separately
        desired_posX = target[0]
        desired_posY = target[1]
        desired_posZ = target[2]

        # Check if close enough to the target
        if (abs(current_endX-desired_posX) + abs(current_endY-desired_posY)) > THRESHOLD:
            # Create vectors to the current and target positions
            current_vectorX = current_endX - root_posX
            current_vectorY = current_endY - root_posY
            current_vectorZ = current_endZ - root_posZ
            current_vector = np.array([current_vectorX, current_vectorY, current_vectorZ])

            target_vectorX = desired_posX - root_posX
            target_vectorY = desired_posY - root_posY
            target_vectorZ = 0.0
            target_vector = np.array([target_vectorX, target_vectorY, target_vectorZ])

            # Normalize vectors
            normalized_current = current_vector/np.linalg.norm(current_vector)
            normalized_target = target_vector/np.linalg.norm(target_vector)

            # Get the Cosine of the target angle by calculating the dot product
            cos_angle = np.dot(normalized_target, normalized_current)

            # Check if cosine is 1
            if cos_angle < 0.99999:
                # Calculate cross product to see which way to rotate
                cross_result = np.cross(normalized_target, normalized_current)

                if cross_result[2] > 0.0: # rotate clockwise
                    # Use arc cosine to get the angle
                    turn_angle = np.arccos(cos_angle)
                    # convert angle to degrees
                    turn_degrees = math.degrees(turn_angle)
                    # Check if degrees are bigger than constraint

                    if turn_degrees > CONSTRAINT:
                        # If so, degrees = constraint
                        turn_angle = math.radians(CONSTRAINT)
                    self.angle += turn_angle

                elif cross_result[2] < 0.0: # rotate counter-clockwise
                    # Use arc cosine to get the angle
                    turn_angle = np.arccos(cos_angle)
                    # convert angle to degrees
                    turn_degrees = math.degrees(turn_angle)
                    # Check if degrees are bigger than constraint

                    if turn_degrees > CONSTRAINT:
                        # If so, degrees = constraint
                        turn_angle = math.radians(CONSTRAINT)

                    self.angle -= turn_angle

class robot_arm:
    def __init__(self) -> None:
        """Set up the basic parameters of the arm."""
        self.end_position = np.array([0.0, 0.0, 0.0]) # Third dimension for cross product
        self.segments = []
    
    def add_segment(self,arm_segment):
        """Add a segment to the arm."""
        self.segments.append(arm_segment)

    def calc_endpoint(self):
        """Calculate the end position of the arm."""
        self.end_position = np.array([0.0,0.0,0.0]) # Initial mountpoint
        self.end_angle = 0.0 # Initial angle in radians
        for s in self.segments:
            s.set_mount_position(self.end_position, self.end_angle)
            s.calc_endpoint()
            self.end_position = s.get_endpoint()
            self.end_angle = s.get_combined_angle()

    def plot(self):
        """Plot the arm."""
        self.calc_endpoint()
        create_figure()
        for s in self.segments:
            s.plot()

    def on_click(self, event):
        """Check if the plot has been clicked. If so, move the arm."""
        if event.button == 1: # Check if the plot is clicked
            target = np.array([event.xdata, event.ydata, 0.0]) # Get the coordinates
            self.move_to(target) # Move the arm to the coordinates
            # Print the target and arm coordinates
            print("Target:", target)
            self.print()

    def move_to(self,pos):
        """Move the arm the the given position.
        pos : np.array
            x, y and z coords of the target position
        """
        maximum_loopcount = 20
        while  np.array_equal(pos,self.end_position) == False: # Check if target and current position are the same
            for seg in reversed(self.segments): # Start moving the arm, beginning with the last segment
                seg.rotate_to(pos,self.end_position) # Rotate the arm
                self.calc_endpoint() # Update coordinates of the arm
                #self.plot()

            #print("Iteration:", maximum_loopcount)
            maximum_loopcount -= 1
            if maximum_loopcount == 0: break

    def print(self):
        """Print the data of individual arm segments."""
        self.calc_endpoint() # Update coordinates of the arm
        for s in self.segments:
            s.print() # Print data of the arm segments
        self.plot()
        print('End angle: {0}'.format(math.degrees(self.end_angle))) # Print final angle of the arm
        print("")

# Create arm object and add segments
arm = robot_arm()
arm.add_segment(arm_segment(20, 45))
arm.add_segment(arm_segment(15, 45))
arm.add_segment(arm_segment(10, 45))
arm.add_segment(arm_segment(10, -45))

# Initial arm plot and print
create_figure()
arm.plot()
arm.print()

# Enable button press events
plt.connect('button_press_event', arm.on_click)

plt.show()