import numpy as np
import random
from NavigateControl import NavigateControl

dir_sensors = {'up': ['left', 'up', 'right'], 'right': ['up', 'right', 'down'],
               'down': ['right', 'down', 'left'], 'left': ['down', 'left', 'up']}
dir_move = {'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0],'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'down', 'right': 'left', 'down': 'up', 'left': 'right'}
			   
dir_heading = {(0,1):'up',(1,0):'right',(0,-1):'down',(-1,0):'left'}
class Robot(object):
	
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.time=0
        self.flag=1
        self.control = NavigateControl(maze_dim,self.flag)        
        self.exploratory=True

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        #if self.control.if_found():
          #self.control.get_chk2()
        if self.time > self.maze_dim * self.maze_dim and self.exploratory==True and self.flag==1:
           self.reset_robot()
           return 'Reset','Reset'
		   
        elif self.flag == 0 and self.control.if_found() and self.exploratory==True:
           self.reset_robot()
           return 'Reset','Reset'
        """if self.exploratory == True and self.control.if_found() and self.control.is_optimal():
           self.reset_robot()
           return 'Reset','Reset'"""
		
        if self.exploratory==True:
           self.control.update_current_loc(self.location,sensors,self.heading)		
        
        

        
    	  
        
           
        
        #print("Current:",self.location)
        
        
        
        next_cell = self.control.get_next_movement(self.location)
                
        
        
        
        intended_dir = self.get_intended_direction(list(next_cell))
        
        rotation, movement = self.get_movement(intended_dir,list(next_cell))
      
		
        
        #print("time: ", self.time)
        #print("Rotation: ", rotation)
        #print("Move: ", movement)
        #print("Intended: ",intended_dir)
        #print("Location:", self.location)
        #print("Heading: ", self.heading)
        #print("Nxt: ",next_cell)
        ##print(self.visited)
        #print(self.control.value_matrix)
        #print(self.graph)
        
        #print("prev after optimize_travel",self.prev_cell)
        self.update_robot(rotation,movement)
        #print("prev after update_robot",self.prev_cell)
        self.time+=1
        #print("self.end at last", self.end_goal)
        #print("-------------------")
        return rotation, movement
	
	
    	

    

	
    
    
	
    def reset_robot(self):
        self.location=[0,0]
        self.heading='up'
        self.time=0
        self.control.waypoints=[]
        print(self.control.visited)
        print(self.control.value_matrix)
        self.control.patch_cell()
        self.control.final_update()
        self.exploratory=False
        print(self.control.visited)      
        path = self.control.modified_djkstra2(tuple(self.location))
        #print("Path:",path)
        self.control.get_waypoints(path)
        #print("waypoints:",self.control.waypoints)
    
	
	
    def update_robot(self,rotation,movement):
        # perform rotation
        if rotation == -90:
           self.heading = dir_sensors[self.heading][0]
        elif rotation == 90:
           self.heading = dir_sensors[self.heading][2]
		   
        while movement:
            if movement > 0:
               self.location[0] += dir_move[self.heading][0]
               self.location[1] += dir_move[self.heading][1]
               movement -= 1
            else:
               rev_heading = dir_reverse[self.heading]
               self.location[0] += dir_move[rev_heading][0]
               self.location[1] += dir_move[rev_heading][1]
               movement += 1
	
    def get_movement(self,intended_dir,next_cell):
        intended_vec=dir_move[intended_dir]
        heading_vec = dir_move[self.heading]
        steps = abs(self.location[0]-next_cell[0]) + abs(self.location[1]-next_cell[1])
        prod = np.dot(intended_vec,heading_vec)
        if prod == 1:
           return 0,steps
        if prod == -1 and self.control.visited[tuple(next_cell)]==0:
           return 90,0
        if prod == -1 and self.control.visited[tuple(next_cell)]==1:
           return 0,-1*steps
        if heading_vec[0]==0:
           if heading_vec[1]*intended_vec[0]==1:
              return 90,steps
           else:
              return -90,steps
        else:
           if heading_vec[0]*intended_vec[1]==-1:
              return 90,steps
           else:
              return -90,steps
		
	
    def get_intended_direction(self,next_cell):
        vec=[]
        vec.append(next_cell[0]-self.location[0])
        vec.append(next_cell[1]-self.location[1])
        norm = abs(vec[0])+abs(vec[1])
        vec[0]=vec[0]/norm
        vec[1]=vec[1]/norm
        """
        if [self.location[0]+0,self.location[1]+1] == next_cell:
           return 'up'
        if [self.location[0]+1,self.location[1]+0] == next_cell:
           return 'right'
        if [self.location[0]+0,self.location[1]-1] == next_cell:
           return 'down'
        if [self.location[0]-1,self.location[1]+0] == next_cell:
           return 'left'
        """
        if vec == [0,1]:
           return 'up'
        if vec == [1,0]:
           return 'right'
        if vec == [0,-1]:
           return 'down'
        if vec == [-1,0]:
           return 'left'
        #print(self.location)
        #print(next_cell)
			
	
    
		
    
        

	
    
			
    		
    
    
		
        
				
   	
	
 
    
	
    
		
    
		
    

    
    	

    	  
#sample_robot = Robot(6)
#print(sample_robot.get_waypoints([(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(2,6),(3,6),(3,7)]))