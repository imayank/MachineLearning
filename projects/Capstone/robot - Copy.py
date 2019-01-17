import numpy as np
import random

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
        central = maze_dim//2
        self.goal = [(central,central),(central-1,central),(central,central-1),(central-1,central-1)]
        self.value_matrix = np.zeros((maze_dim,maze_dim))
        self.graph = {}
        self.visited = np.zeros((maze_dim,maze_dim))
        self.visit_count = np.zeros((maze_dim,maze_dim))
        self.found=False
        self.time=0
        self.exploratory = True
        self.exploitatory = False
        self.chkpoints = []
        self.waypoints=[]
        self.forward= True
        self.next_chkpoint = (0,0)
        self.end=[]
        for i in range(maze_dim):
           for j in range(maze_dim):
              manhattan_dist=[]
              for cell in self.goal:
                 manhattan_dist.append((abs(i-cell[0])+abs(j-cell[1])))
              self.value_matrix[i,j] = min(manhattan_dist)
        
    

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
        
		
		
        if len(self.waypoints) > 0:
           next_cell=self.waypoints.pop(0)
           print(next_cell)
           print(self.location)
           intended_dir = self.get_intended_direction(list(next_cell))
           print(intended_dir)
           rotation, movement = self.get_movement(intended_dir,list(next_cell))
           self.update_robot(rotation,movement)
           self.time+=1
           self.visit_count[tuple(self.location)]+=1
           return rotation, movement
		
		
        self.visit_count[tuple(self.location)]+=1
		
        if tuple(self.location) in self.goal and self.forward==True:
           self.found=True		
           self.end = self.location
           if self.visited[tuple(self.location)] == 0:
             self.graph_update(sensors)
             self.visited[self.location[0],self.location[1]]=1
           self.propogate_values(self.location)
           
           self.populate_checkpoints()
           if len(self.chkpoints)==0:
             self.exploratory = False		  
           else:
             print("Chkpoints: ", self.chkpoints)
             print(self.visited)
             chk = self.get_important_checkpoint()
             self.next_chkpoint=chk
             print("Chk:",chk)
             #print(self.value_matrix)
             #path = self.modified_djkstra(tuple(self.location),tuple(chk))
             #self.get_waypoints(path)
             self.forward=False
             self.update_robot(90,0)  ###### correcting orientialtion
             return 90,0			 
        		
        
        if self.exploratory != True:
           self.reset_robot()
           return 'Reset','Reset'

        
    	  
        if self.visited[tuple(self.location)] == 0:
           self.graph_update(sensors)
           self.visited[self.location[0],self.location[1]]=1
           
        nbrs = self.get_neighbours(self.location)
		
        if self.forward==True:
          next_cell = self.get_nxt_cell(nbrs)
         
          if next_cell == None:
            print("CHANGE!!!")
            stk=[self.location]
            #print(self.value_matrix)
            self.value_update(stk)
            #print(self.value_matrix)
            next_cell = self.get_nxt_cell(nbrs)
            
        else:
            candidate = []
            prev_dir = dir_move[dir_reverse[self.heading]]
            for nbh in nbrs:
              if self.visited[tuple(nbh)] == 1 and tuple(nbh) not in self.goal:
                if self.value_matrix[tuple(nbh)] >= self.value_matrix[tuple(self.location)]+1:
                  self.value_matrix[tuple(nbh)] = self.value_matrix[tuple(self.location)]+1
                if nbh[0]!=self.location[0]+prev_dir[0] or nbh[1] != self.location[1]+prev_dir[1]:
                  candidate.append(tuple(nbh))
              elif self.visited[tuple(nbh)] == 0 and tuple(nbh) not in self.goal:
                self.value_matrix[tuple(nbh)] = self.value_matrix[tuple(self.location)]+1
                candidate.append(tuple(nbh))
				
            print("Before_cand:",candidate)
            if len(candidate)==0:
              candidate=[nbh for nbh in nbrs if tuple(nbh) not in self.goal]
            print("Candidate:", candidate)
            next_cell = self.reverse_next_cell(candidate)
            if tuple(next_cell) == tuple(self.next_chkpoint):
              self.forward=True
              self.propogate_values(self.end)
              print("Forward")			  
         
        """
        if self.visited[tuple(next_cell)] == 1 and self.found == True and self.visit_count[tuple(next_cell)] > 1:
           self.path_to_chkpoint()
           print(self.exploratory)
           if self.exploratory == False:
            self.reset_robot()
            return 'Reset','Reset'
           print("Location: ",self.location," Waypoint: ",self.waypoints[0])
           intd_dir = self.get_intended_direction(list(self.waypoints[0]))
           rot, mv = self.get_movement(intd_dir,list(self.waypoints[0]))
           if mv==0:
            self.update_robot(90,0)
            return 90,0
           else:
            rotation, movement = self.goto_nxt_waypoint()
            return rotation, movement"""
        
        intended_dir = self.get_intended_direction(list(next_cell))
        
        rotation, movement = self.get_movement(intended_dir,list(next_cell))
        print("-------------------")
        print("time: ", self.time)
        print("Rotation: ", rotation)
        print("Move: ", movement)
        print("Intended: ",intended_dir)
        print("Location:", self.location)
        print("Heading: ", self.heading)
        print(self.visit_count)
        print(self.value_matrix)
        #print(self.graph)
        
        self.update_robot(rotation,movement)
        self.time+=1
        return rotation, movement
	
	
    """def propogate_values(self,cell):
        lst=[tuple(cell)]
        examined=[]
        #print("Propogating")
        while len(lst) != 0:
          curr_node = lst.pop(0)
          #print(curr_node)
          for nbr in self.graph[tuple(curr_node)]:
            #print(self.visited[nbr],nbr)
            if nbr not in examined and nbr not in self.goal:
              if self.visited[nbr] == 1.0:
               
               self.value_matrix[nbr] = self.value_matrix[curr_node]+1
               #print(curr_node, "-->",nbr," value--->",self.value_matrix[nbr])
               lst.append(nbr)
          examined.append(curr_node)
        print(self.value_matrix)"""
    def reverse_next_cell(self,nbrs):
	
        visited_nbr = [nbh for nbh in nbrs if self.visited[tuple(nbh)] == 1]
        unvisited_nbr = [nbh for nbh in nbrs if self.visited[tuple(nbh)] == 0]
		
        
        if len(unvisited_nbr) > 0:
          min_unvisited = min([abs(nbh[0]-self.next_chkpoint[0])+abs(nbh[1]-self.next_chkpoint[1]) for nbh in unvisited_nbr])
        if len(visited_nbr) > 0:  
          min_visited = min([abs(nbh[0]-self.next_chkpoint[0])+abs(nbh[1]-self.next_chkpoint[1]) for nbh in visited_nbr])
		
        nxt_unvisited = []
        nxt_visited = []
		
        for nbh in unvisited_nbr:
          if abs(nbh[0]-self.next_chkpoint[0])+abs(nbh[1]-self.next_chkpoint[1]) == min_unvisited:
            nxt_unvisited.append(nbh)
        for nbh in visited_nbr:
          if abs(nbh[0]-self.next_chkpoint[0])+abs(nbh[1]-self.next_chkpoint[1]) == min_visited:
            nxt_visited.append(nbh)
			
        if len(unvisited_nbr) > 0:
           return random.choice(nxt_unvisited)
        else:
           return random.choice(nxt_visited)
		
    def propogate_values(self,cell):
        lst = [tuple(cell)]
        examined=[]
        while len(lst) != 0:
          current_cell = lst.pop(0)
          neighbours = self.get_neighbours(current_cell)
          curr_cell_value = self.value_matrix[current_cell[0],current_cell[1]]
          #print("Examined: ",examined)
          #print("List: ",lst)
          for nbh in neighbours:
            #print("Nbh: ",nbh)
            if tuple(nbh) not in examined and tuple(nbh) not in self.goal and tuple(nbh) not in lst:
               if self.visited[current_cell] == 1:
                 self.value_matrix[tuple(nbh)] = curr_cell_value +1
                 lst.append(tuple(nbh))
               elif self.visited[tuple(nbh)] == 0:
                 self.value_matrix[tuple(nbh)] = curr_cell_value +1
                 lst.append(tuple(nbh))
          examined.append(current_cell)
        print("after_propogation",self.value_matrix)		  
	
    def reset_robot(self):
        self.exploitatory = True
        self.location=[0,0]
        self.heading='up'
        self.time=0
        self.waypoints=[]
    
        for nd in self.goal:
          if self.visited[int(nd[0]),int(nd[1])] == 1:
           tg = nd
           break
              
        path = self.modified_djkstra(tuple(self.location),tuple(tg))
        self.get_waypoints(path)
              
        print(self.visit_count)
        print(self.value_matrix)
        #return 'Reset','Reset'
	
    def path_to_chkpoint(self):
        self.populate_checkpoints()
        #self.chkcount+=1
        if len(self.chkpoints)==0:
           
           self.exploratory = False
        else:
           print("Chkpoints: ", self.chkpoints)
           print(self.visited)
           chk = self.get_important_checkpoint()
           path = self.modified_djkstra(tuple(self.location),tuple(chk))
           self.get_waypoints(path)
           
    def goto_nxt_waypoint(self):
        next_cell=self.waypoints.pop(0)
        intended_dir = self.get_intended_direction(list(next_cell))
        rotation, movement = self.get_movement(intended_dir,list(next_cell))
        self.update_robot(rotation,movement)
        self.time+=1
        return rotation, movement   
	
	
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
        if prod == -1:
           return 90,0#-1*steps
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
			
	
    def get_nxt_cell(self,nbrs):
        min_value=min([self.value_matrix[tuple(nbh)] for nbh in nbrs])
        if min_value != self.value_matrix[tuple(self.location)]-1:
           return None
        nxt_visited=[]
        nxt_unvisited=[]
        for nb in nbrs:
           if self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 0:
             nxt_unvisited.append(nb)
           elif self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 1:
             nxt_visited.append(nb)
             
        if len(nxt_unvisited) > 0:
           return random.choice(nxt_unvisited)
        else:
           return random.choice(nxt_visited)	 
	
    def graph_update(self,sensors):
        self.graph[tuple(self.location)]=[]
        for i in range(len(sensors)):
           if sensors[i] > 0:
             nb=dir_sensors[self.heading][i]
             self.graph[tuple(self.location)].append((self.location[0]+dir_move[nb][0],self.location[1]+dir_move[nb][1]))
        last=list(set(['left','up','right','down'])-set(dir_sensors[self.heading]))[0]
        elm=[]
        elm.append(self.location[0]+dir_move[last][0])
        elm.append(self.location[1]+dir_move[last][1])
        if elm[0] >= 0 and elm[1] >=0 and elm[0] < self.maze_dim and elm[1] < self.maze_dim:
           self.graph[tuple(self.location)].append((elm[0],elm[1]))

	
    def get_neighbours(self,grid_cell):
        if self.visited[grid_cell[0],grid_cell[1]] == 1:
           return self.graph[tuple(grid_cell)]
        nbhs = []
        for direction in [(0,1),(1,0),(0,-1),(-1,0)]:
            elm=[]
            elm.append(grid_cell[0]+direction[0])
            elm.append(grid_cell[1]+direction[1])
            if elm[0] >= 0 and elm[1] >=0 and elm[0] < self.maze_dim and elm[1] < self.maze_dim:
               nbhs.append(elm)
        return nbhs
			
    		
    
	
    def value_update(self,stack):
        while len(stack) != 0:
             current_cell = stack.pop()
             neighbours = self.get_neighbours(current_cell)
             min_value = min([self.value_matrix[nbh[0],nbh[1]] for nbh in neighbours])
             curr_cell_value = self.value_matrix[current_cell[0],current_cell[1]]
             if min_value != curr_cell_value-1:
                self.value_matrix[current_cell[0],current_cell[1]]=min_value+1
                for nbh in neighbours:
                    if tuple(nbh) not in self.goal:
                     stack.append(nbh)				
		
    def populate_checkpoints(self):
        #if self.visited[tuple(self.location)] == 1:
        #self.chkpoints = []  ### over time checkpoint cease to be a checkoint
        print(self.chkpoints)
        rem = []
        #if self.visited[tuple(self.location)] == 1:
           
        for chk in self.chkpoints:
             
          min_value = min([self.value_matrix[tuple(nb)] for nb in self.graph[tuple(chk)]]) 
          to_remove=0  
          for nb in self.graph[tuple(chk)]:
            if self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 1:
              to_remove=1
              #rem.append(tuple(chk))
            elif self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 0:
              to_remove=0 
              break
          if to_remove == 1:
             rem.append(tuple(chk))
        for chk in rem:
          self.chkpoints.remove(chk)
        """
        if self.trial_flag == True:
           return
        else:
           self.trial_flag = True
           self.chkcount+=1
        """  		   
        for node in self.graph:
           if node not in self.goal and self.visit_count[node] <= 2:  #### goal can't be a checkpoint
            nbrs = self.graph[node]
            min_value = min([self.value_matrix[tuple(nb)] for nb in nbrs])
            for nb in nbrs:
               if self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 0:
                 nxt_min_value = min([self.value_matrix[tuple(cell)] for cell in self.get_neighbours(nb)])
                 if nxt_min_value < min_value:
                  for cell in self.get_neighbours(nb):
                    if self.value_matrix[tuple(cell)] == nxt_min_value and self.visited[tuple(cell)] == 0:
                      if(tuple(node) not in self.chkpoints):
                        self.chkpoints.append(tuple(node))
                   #break
        
				
    def shortest_path(self,src,target):
        #src and target should be tuple
        #unexamined = [tuple(node) for node in self.graph]
        to_examin=[src]
        examined=[]
        dist_src = {}
        for node in self.graph:
           dist_src[tuple(node)] = 100000
        dist_src[src]=0
		#curr_exam=src
        while target not in examined:
             #print(dist_src)
             min_value=min([dist_src[node] for node in to_examin])
             for node in to_examin:
                if dist_src[node]==min_value:
                  curr_exam=node
                  break
			 
             for nbr in self.graph[curr_exam]:
                if self.visited[tuple(nbr)] == 1 and dist_src[nbr] > dist_src[curr_exam]+1:
                   dist_src[nbr] = dist_src[curr_exam]+1
                   if nbr not in examined:
                     to_examin.append(nbr)
             to_examin.remove(curr_exam)
             examined.append(curr_exam)			 
             
                
        path=[]
        curr_vertex = target
        while True:
             path.append(curr_vertex)
             for nbr in self.graph[curr_vertex]:
                if self.visited[tuple(nbr)] == 1 and dist_src[nbr] == dist_src[curr_vertex]-1:
                   curr_vertex = nbr
             if tuple(curr_vertex) == tuple(src):
                break
        path.append(src)
        #print(path)
        path.reverse()
        return path		
	
    def get_waypoints(self,path):
        self.waypoints=[]
        i=0
        while i < len(path)-1:
             move_dir = [ path[i+1][0]-path[i][0],path[i+1][1]-path[i][1]]
             for j in range(1,4):
                if i+1< len(path) and path[i][0]+move_dir[0] == path[i+1][0] and path[i][1]+move_dir[1] == path[i+1][1]:
                  i+=1
                else:
                  break
                
             self.waypoints.append(path[i])
        #return waypoints
    def get_nearest_checkpoint(self):
        min_value = min([abs(self.location[0]-chk[0])+abs(self.location[1]-chk[1]) for chk in self.chkpoints])
        for chk in self.chkpoints:
           if abs(self.location[0]-chk[0])+abs(self.location[1]-chk[1]) == min_value:
             return chk
	
    def get_important_checkpoint(self):
        #max_nbrs = max([len(self.graph[tuple(node)]) for node in self.chkpoints])
        max_value=0
        for node in self.chkpoints:
           count=0
           for nbr in self.graph[tuple(node)]:
              #print("Neighbour: ",nbr)
              #print(self.visited[tuple(nbr)])
              if self.visited[tuple(nbr)] == 0:
                count+=1 
              print(count)
           if count > max_value:
             max_value = count
             max_node = node			 
        #for chk in self.chkpoints:
           #if len(self.graph[tuple(chk)]) == max_nbrs:
             #return chk
        return max_node
		
    
    def modified_djkstra(self,src,target):
        to_examin=[src]
        examined=[]
        dist_src = {}
        for node in self.graph:
           dist_src[tuple(node)] = [100000,100000,'nope']
        dist_src[src]=[1,0,'up'] #### (step_value, total_prev_value, heading)
        curr_head='up'
        next_value=0
		#curr_exam=src
        cycle = 0
        while target not in examined:
             #print(dist_src)
             min_value=min([dist_src[node][0] + dist_src[node][1] for node in to_examin])
             for node in to_examin:
                if dist_src[node][0] + dist_src[node][1]==min_value:
                  curr_exam=node
                  break
            
             #if dist_src[curr_exam] == 1:
			 
             for nbr in self.graph[curr_exam]:
                if self.visited[tuple(nbr)] == 1 and nbr not in examined:
                  vec=[nbr[0]-curr_exam[0],nbr[1]-curr_exam[1]]
                  if dir_move[dist_src[curr_exam][2]] == vec and dist_src[curr_exam][0] in [1,0.5,0.25]: #cycle > 0:
                    next_value = dist_src[curr_exam][0]/2
                    next_heading = dist_src[curr_exam][2]
                    if curr_exam != src:
                      dist_src[curr_exam][0] = next_value
                    #cycle=(cycle+1)%3
                  else:
                    next_value = 1
                    next_heading = 	dir_heading[tuple(vec)]
                    #cycle=1	
					
                   
                  dist_src[nbr][0] = next_value
                  dist_src[nbr][1] = dist_src[curr_exam][0] + dist_src[curr_exam][1]
                  dist_src[nbr][2] = next_heading
				  
                  #print("Current: ", dist_src[curr_exam],curr_exam)
                  #print("Nbr appended",dist_src[nbr],nbr)
                  to_examin.append(nbr)
             #print("-------------------------------")	  
             to_examin.remove(curr_exam)
             examined.append(curr_exam) 

        path=[]
        curr_vertex = target
        #for item in dist_src.items():
        #   print(item)
        print("Source: ",src, " Target: ", target)
        while True:
             path.append(curr_vertex)

             min_value=dist_src[curr_vertex][0] + dist_src[curr_vertex][1]
             for nbr in self.graph[curr_vertex]:
                if self.visited[tuple(nbr)] == 1 and dist_src[nbr][0] + dist_src[nbr][1] < min_value:

                   prob_vertex = nbr
                   min_value = dist_src[nbr][0] + dist_src[nbr][1]
             curr_vertex = prob_vertex
             if tuple(curr_vertex) == tuple(src):
                break
        path.append(src)
        #print(path)
        path.reverse()
        return path						
      	
#sample_robot = Robot(6)
#print(sample_robot.get_waypoints([(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(2,6),(3,6),(3,7)]))