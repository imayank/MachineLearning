import numpy as np
import random

dir_sensors = {'up': ['left', 'up', 'right'], 'right': ['up', 'right', 'down'],
               'down': ['right', 'down', 'left'], 'left': ['down', 'left', 'up']}
			   
dir_move = {'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0],'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],}

dir_heading = {(0,1):'up',(1,0):'right',(0,-1):'down',(-1,0):'left'}

class NavigateControl(object):

    def __init__(self,maze_dim,flag):
	 
        self.maze_dim = maze_dim
        self.flag = flag
        central = maze_dim//2
        self.goal = [(central,central),(central-1,central),(central,central-1),(central-1,central-1)]
        self.value_matrix = np.zeros((maze_dim,maze_dim))
        self.graph = {}
        self.visited = np.zeros((maze_dim,maze_dim))
        self.visit_count = np.zeros((maze_dim,maze_dim))
        self.found=tuple()
        self.waypoints=[]
        self.forward= True
        self.prev_cell=None
        self.trgt = tuple()
        for i in range(maze_dim):
           for j in range(maze_dim):
              manhattan_dist=[]
              for cell in self.goal:
                 manhattan_dist.append((abs(i-cell[0])+abs(j-cell[1])))
              self.value_matrix[i,j] = min(manhattan_dist)
	
    def update_current_loc(self,loc,sensors,heading):
        self.visit_count[tuple(loc)]+=1
        if self.visited[tuple(loc)] == 0:
           self.graph_update(sensors,loc,heading)
           self.visited[loc[0],loc[1]]=1
        self.prev_cell=tuple(loc)
	
    def graph_update(self,sensors,loc,heading):
      
        for i in range(len(sensors)):
           if sensors[i] > 0:
             k=sensors[i]
             nb=dir_sensors[heading][i]
             cell = tuple(loc)
             #print("nb and k",nb,k)
             while k > 0:
               if cell not in self.graph:
                 self.graph[cell]=[]
               elm=(cell[0]+dir_move[nb][0],cell[1]+dir_move[nb][1])
               if elm not in self.graph[cell]:
                 self.graph[cell].append(elm)
               if elm not in self.graph:
                 self.graph[elm]=[]
                 self.graph[elm].append(cell)
               elif tuple(cell) not in self.graph[elm]:
                 self.graph[elm].append(cell)
               k=k-1
               cell=elm
        if self.prev_cell != None and self.prev_cell not in self.graph[tuple(loc)]:
           self.graph[tuple(self.loc)].append((self.prev_cell[0],self.prev_cell[1]))
	
    def get_next_movement(self,loc):
	
        if len(self.waypoints) > 0:
           return self.waypoints.pop(0)
	
        if self.flag == 0:
           return self.random_movement(loc)
        else:
           return self.algorithmic_movement(loc)
		   
		   
    def random_movement(self,cell):
        if tuple(cell) in self.goal:
          self.found=cell
        nbrs = self.get_neighbours(cell)
        nxt_visit = []
        nxt_unvisit = []
        for nb in nbrs:
           if self.visited[tuple(nb)] == 1:
             nxt_visit.append(nb)
           else:
             nxt_unvisit.append(nb)
        if len(nxt_unvisit) > 0:
           return random.choice(nxt_unvisit)
        else:
           return random.choice(nxt_visit)
		   
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
		
    def algorithmic_movement(self,loc):
	
        if tuple(loc) in self.goal and self.forward == True:
           self.found=tuple(loc)		
           self.trgt=self.get_chk2()
           self.matrix_update(tuple(self.trgt))
           self.forward=False
           #print("########## Now Reverse ############\n")
        elif tuple(loc) == tuple(self.trgt) and self.forward == False:
            self.matrix_update(self.found)
            self.forward = True
            self.trgt = tuple(self.found)
            #print("########## Now Forward ############\n")
        
        nbrs=self.get_neighbours(loc)
        next_cell = self.get_nxt_cell(nbrs,loc)
		
        if next_cell == None:
          #print("CHANGE!!!")
          stk=[loc]
          self.value_update(stk)
          next_cell = self.get_nxt_cell(nbrs,loc)
		  
        if self.visited[tuple(next_cell)] == 1:
           self.optimize_travel(next_cell)
		  
        return next_cell
          
			
    def get_chk2(self):
        chk_lst=[]
        for node in self.graph:
           if node not in self.goal  and self.visit_count[node]!=0:  #### goal can't be a checkpoint
            nbrs = self.graph[node]
            min_value = min([self.value_matrix[tuple(nb)] for nb in nbrs])
            for nb in nbrs:
               if self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 0:
                  if(tuple(node) not in chk_lst):
                    chk_lst.append(tuple(node))
        #return self.get_important_checkpoint(chk_lst)
        #print(chk_lst)
        return self.get_nearest_checkpoint(chk_lst)
		
    def get_nearest_checkpoint(self,lst_chkpoints):
        min_value = min([abs(chk[0])+abs(chk[1]) for chk in lst_chkpoints])
        for chk in lst_chkpoints:
           if abs(chk[0])+abs(chk[1]) == min_value:
             #print("list and checkpoint:",lst_chkpoints,chk)
             return chk
	
    def matrix_update(self,cell):
	
        for i in range(self.maze_dim):
           for j in range(self.maze_dim):
              self.value_matrix[i,j] = 300		   
  
        self.value_matrix[cell] = 0
        
        lst = [tuple(cell)]
        examined=[]
        while len(lst) != 0:
          current_cell = lst.pop(0)
          neighbours = self.get_neighbours(current_cell)
          curr_cell_value = self.value_matrix[current_cell[0],current_cell[1]]
          #print("Examined: ",examined)
          #print("List: ",lst)
          #print("Curr Cell: ",current_cell)
          #print("Curr value: ",curr_cell_value)
          for nbh in neighbours:
            #print("Nbh: ",nbh)
            if tuple(nbh) not in examined:
               val = curr_cell_value + 1
               if self.visited[current_cell] == 1:
                 if self.value_matrix[tuple(nbh)] > val:
                   self.value_matrix[tuple(nbh)] = val
                 #elif tuple(nbh) not in lst:
                   #self.value_matrix[tuple(nbh)] = val
                 if tuple(nbh) not in lst:
                   lst.append(tuple(nbh))
               elif self.visited[tuple(nbh)] == 0:
                 if self.value_matrix[tuple(nbh)] > val:
                   self.value_matrix[tuple(nbh)] = val
                 if tuple(nbh) not in lst:
                   lst.append(tuple(nbh))
               elif self.visited[tuple(nbh)] == 1 and tuple(current_cell) in self.graph[tuple(nbh)]:
                 if self.value_matrix[tuple(nbh)] > val:
                   self.value_matrix[tuple(nbh)] = val
                 if tuple(nbh) not in lst:
                   lst.append(tuple(nbh))
                 #elif tuple(nbh) not in lst:
                   #self.value_matrix[tuple(nbh)] = val
               
          examined.append(current_cell)
		
        if self.forward == True:
           for loc in self.goal:
              if loc != self.found:
                self.value_matrix[loc] = 300
        else:
           for loc in self.goal:
              self.value_matrix[loc] = 0
		
        #print("Matrix_update\n",self.value_matrix)
        ##print(self.visited)

    def get_nxt_cell(self,nbrs,loc):
        min_value=min([self.value_matrix[tuple(nbh)] for nbh in nbrs])
        if min_value != self.value_matrix[tuple(loc)]-1:
           return None
        nxt_visited=[]
        nxt_unvisited=[]
        temp=[]
        for nb in nbrs:
           if self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 0:
             nxt_unvisited.append(nb)
           elif self.value_matrix[tuple(nb)] == min_value and self.visited[tuple(nb)] == 1:
             nxt_visited.append(nb)
           elif self.value_matrix[tuple(nb)] == min_value+1 and self.visited[tuple(nb)] == 0:
             temp.append(nb)
             
        if len(nxt_unvisited) > 0:
           #print("Unvisited:",len(nxt_unvisited),self.forward)
           #return random.choice(nxt_unvisited)
           return self.choose_nxt(nxt_unvisited,loc)
        elif len(temp) > 0:
           return random.choice(temp)
        else:
           #print("Visited:",len(nxt_visited),self.forward)
           #return random.choice(nxt_visited)
           return self.choose_nxt_visited(nxt_visited)		   
	
    def choose_nxt_visited(self,lst):
        if len(lst) == 1:
          return lst[0]
		  
        min_val = min([self.visit_count[tuple(cell)] for cell in lst])
        for cell in lst:
           if min_val==self.visit_count[tuple(cell)]:
             return cell
	
    def choose_nxt(self,lst,loc):
        if len(lst) == 1:
           return lst[0]
        reg = self.get_region(loc)
        
        if self.forward == True:
           if reg == 1 or reg == 2 or reg == 3:
             if (loc[0]+1,loc[1]) in lst:
                return (loc[0]+1,loc[1])
             else:
                #print("random")
                #return random.choice(lst)
                return self.best_frontier(lst)
           elif reg == 8 or reg == 7:
             if (loc[0],loc[1]+1) in lst:
                return (loc[0],loc[1]+1)
             else:
                #print("random")
                #return random.choice(lst)
                return self.best_frontier(lst)
           elif reg == 4 or reg == 5:
             if (loc[0],loc[1]-1) in lst:
                return (loc[0],loc[1]-1)
             else:
                #print("random")
                #return random.choice(lst)
                return self.best_frontier(lst)
           else:
             if (loc[0]-1,loc[1]) in lst:
                return (loc[0]-1,loc[1])
             else:
                #print("random")
                #return random.choice(lst)
                return self.best_frontier(lst)
        else:
            d = []
            d.append(dir_heading[lst[0][0]-loc[0],lst[0][1]-loc[1]])
            d.append(dir_heading[lst[1][0]-loc[0],lst[1][1]-loc[1]])
            
            if 'down' in d:
              return lst[d.index('down')]
            elif 'left' in d:
              return lst[d.index('left')]
            else:
              #print("random")
              #return random.choice(lst)
              return self.best_frontier(lst)			  
    def get_region(self,cell):
        mid = self.maze_dim//2
        if cell[0] < mid-1:
          if cell[1] < mid-1:
            return 1
          elif cell[1] >=mid-1 and cell[1]<=mid:
            return 2
          else:
            return 3
        elif cell[0] >=mid-1 and cell[0]<=mid:
          if cell[1] < mid-1:
             return 8
          else:
             return 4
        else:
          if cell[1] < mid-1:
            return 7
          elif cell[1] >=mid-1 and cell[1]<=mid:
            return 6
          else:
            return 5

    def best_frontier(self,lst):
        num_unvist = []
        #print(self.visited[0])
        for cell in lst:
           num = 0
           for nb in self.get_neighbours(cell):
              #print(nb,self.visited[tuple(nb)])
              if self.visited[tuple(nb)] == 0:
                num=num+1
           num_unvist.append(num)
           #print("cell and count:",cell,num)
        max_value = max(num_unvist)
        return lst[num_unvist.index(max_value)]

    def all_nbrs(self,grid_cell):
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
                nbrs=self.all_nbrs(current_cell)
                for nbh in nbrs:
                    if tuple(nbh) not in self.goal or tuple(nbh) == self.found:
                     stack.append(nbh)			

    def optimize_travel(self,src):
        path=[src]
        cell = src
        #print("source: ",src)
        while True:
          nxt = self.nxt_loc(cell)
          if nxt == None or self.visited[tuple(nxt)] == 0:
            break
          else:
            path.append(nxt)
            cell = nxt
        if len(path)>1:
          self.get_waypoints(path)
        #print("path: ",path)
        #print("waypoints: ",self.waypoints)
        #print("trgt:",self.trgt)
        if self.trgt in self.waypoints:
           self.reverse_direction()

    def reverse_direction(self):
        #self.waypoints=[]
        if self.trgt in self.goal:
           flag = False
           cell = self.get_chk2()
           self.waypoints=[]
           #print("##Reverse##")
        else:
           flag = True
           cell = tuple(self.found)
           #print("##FoRward##")
        self.matrix_update(cell)
        self.forward=flag
        self.trgt=cell		
	
    def nxt_loc(self,cell):
        nbrs = self.get_neighbours(cell)
        min_value=min([self.value_matrix[tuple(nbh)] for nbh in nbrs])
        if min_value != self.value_matrix[tuple(cell)]-1:
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
           return self.best_frontier(nxt_visited)
						 
    def final_update(self):
        cell = self.found
        for i in range(self.maze_dim):
           for j in range(self.maze_dim):
              self.value_matrix[i,j] = 300		   
  
        self.value_matrix[cell] = 0
        
        lst = [tuple(cell)]
        examined=[]
        while len(lst) != 0:
          current_cell = lst.pop(0)
          neighbours = self.graph[current_cell]
          curr_cell_value = self.value_matrix[current_cell[0],current_cell[1]]
          ##print("Examined: ",examined)
          ##print("List: ",lst)
          #print("Curr Cell: ",current_cell)
          #print("Curr value: ",curr_cell_value)
          for nbh in neighbours:
            #print("Nbh: ",nbh)
            if tuple(nbh) not in examined:
               val = curr_cell_value + 1
               
               if self.value_matrix[tuple(nbh)] > val:
                 self.value_matrix[tuple(nbh)] = val
                 #elif tuple(nbh) not in lst:
                   #self.value_matrix[tuple(nbh)] = val
               if tuple(nbh) not in lst:
                 lst.append(tuple(nbh))
                 #elif tuple(nbh) not in lst:
                   #self.value_matrix[tuple(nbh)] = val
               
          examined.append(current_cell)
        for loc in self.goal:
           self.value_matrix[loc] = 0		  
		   
		   
    def modified_djkstra2(self,src):
        target=tuple(self.found)
        to_examin=[src]
        examined=[]
        dist_src = {}
        for node in self.graph:
           dist_src[tuple(node)] = [100000,100000,'nope']
        dist_src[src]=[0,0,'up'] #### (step_value, total_prev_value, heading)
        curr_head='up'
        next_value=0
		#curr_exam=src
        cycle = 0
        while len(to_examin) > 0:
             #print(dist_src)
             curr_exam=(300,300,'none')
             for node in to_examin:
                if (dist_src[node][0],dist_src[node][1])<(curr_exam[0],curr_exam[1]):
                  curr_exam=node
                  break
            
             #if dist_src[curr_exam] == 1:
             #print("Current:",curr_exam,dist_src[curr_exam])
             flag=True
             for nbr in self.graph[curr_exam]:
                
                val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0],dist_src[curr_exam][1]) 
                if True:
                  vec=[nbr[0]-curr_exam[0],nbr[1]-curr_exam[1]]
                  if dir_move[dist_src[curr_exam][2]] == vec and dist_src[curr_exam][1] in [1,0]:
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0],dist_src[curr_exam][1]+1)
                    if val2:					
                      dist_src[nbr][0] = dist_src[curr_exam][0]
                      dist_src[nbr][1] = dist_src[curr_exam][1]+1
                      dist_src[nbr][2] = dist_src[curr_exam][2]
                      flag=False
                      to_examin.append(nbr)
                  elif dir_move[dist_src[curr_exam][2]] == vec and dist_src[curr_exam][1] == 2:
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0]+1,dist_src[curr_exam][1])
                    if val2:
                      dist_src[nbr][0] = dist_src[curr_exam][0]+1
                      dist_src[nbr][1] = 0
                      dist_src[nbr][2] = dist_src[curr_exam][2]
                      flag=False
                      to_examin.append(nbr)
                  else:
                    val=1
                    if dist_src[curr_exam][0] == 0:
                      val=2
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0]+val,dist_src[curr_exam][1])
                    if val2:
                      dist_src[nbr][0] = dist_src[curr_exam][0]+val
                      dist_src[nbr][1] = 0
                      dist_src[nbr][2] = dir_heading[tuple(vec)]
                      to_examin.append(nbr)
                    #cycle=1
                  if dir_move[dist_src[nbr][2]] == vec and dist_src[nbr][1] in [1,2]:
                    if (nbr[0]+vec[0],nbr[1]+vec[1]) not in self.graph[nbr]:
                      dist_src[nbr][0] = dist_src[nbr][0]
                      #dist_src[nbr][1] = 0
                
                  
                  #print("Nbr:",nbr,dist_src[nbr])				   
             #if flag == True:
                #dist_src[nbr][0]=dist_src[curr_exam][0]+1
                   
                
				  
                  #print("Current: ", dist_src[curr_exam],curr_exam)
                  #print("Nbr appended",dist_src[nbr],nbr)
                 
             #print("-------------------------------")	  
             to_examin.remove(curr_exam)
             examined.append(curr_exam) 

        path=[]
        curr_vertex = target
        #print("Items:")
        #for item in dist_src.items():
           #print(item)
        #print("Source: ",src, " Target: ", target)
        #print("working:",dist_src[self.found])
        while True:
             path.append(curr_vertex)
             #print(curr_vertex,dist_src[curr_vertex][2])
             direct = dir_move[dist_src[curr_vertex][2]]
             mv=(direct[0]*-1,direct[1]*-1)
             curr_vertex = (curr_vertex[0]+mv[0],curr_vertex[1]+mv[1])
             if tuple(curr_vertex) == tuple(src):
                break
        path.append(src)
        #print("this:",path)
        path.reverse()
        return path
		
    def modified_djkstra_open(self,src):
        target=tuple(self.found)
        to_examin=[src]
        examined=[]
        dist_src = {}
        for i in range(self.maze_dim):
           for j in range(self.maze_dim):
              dist_src[(i,j)] = [100000,100000,'nope']
        dist_src[src]=[0,0,'up'] #### (step_value, total_prev_value, heading)
        curr_head='up'
        next_value=0
		#curr_exam=src
        cycle = 0
        while len(to_examin) > 0:
             #print(dist_src)
             curr_exam=(300,300,'none')
             for node in to_examin:
                if (dist_src[node][0],dist_src[node][1])<(curr_exam[0],curr_exam[1]):
                  curr_exam=node
                  break
            
             #if dist_src[curr_exam] == 1:
             #print("Current:",curr_exam,dist_src[curr_exam])
             flag=True
             for nbr in self.get_neighbours(curr_exam):
                nbr = tuple(nbr)
                val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0],dist_src[curr_exam][1]) 
                if True:
                  vec=[nbr[0]-curr_exam[0],nbr[1]-curr_exam[1]]
                  if dir_move[dist_src[curr_exam][2]] == vec and dist_src[curr_exam][1] in [1,0]:
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0],dist_src[curr_exam][1]+1)
                    if val2:					
                      dist_src[nbr][0] = dist_src[curr_exam][0]
                      dist_src[nbr][1] = dist_src[curr_exam][1]+1
                      dist_src[nbr][2] = dist_src[curr_exam][2]
                      flag=False
                      to_examin.append(nbr)
                  elif dir_move[dist_src[curr_exam][2]] == vec and dist_src[curr_exam][1] == 2:
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0]+1,dist_src[curr_exam][1])
                    if val2:
                      dist_src[nbr][0] = dist_src[curr_exam][0]+1
                      dist_src[nbr][1] = 0
                      dist_src[nbr][2] = dist_src[curr_exam][2]
                      flag=False
                      to_examin.append(nbr)
                  else:
                    val=1
                    if dist_src[curr_exam][0] == 0:
                      val=2
                    val2 = (dist_src[nbr][0],dist_src[nbr][1])>(dist_src[curr_exam][0]+val,dist_src[curr_exam][1])
                    if val2:
                      dist_src[nbr][0] = dist_src[curr_exam][0]+val
                      dist_src[nbr][1] = 0
                      dist_src[nbr][2] = dir_heading[tuple(vec)]
                      to_examin.append(nbr)
                    #cycle=1
                  """if dir_move[dist_src[nbr][2]] == vec and dist_src[nbr][1] in [1,2]:
                    if (nbr[0]+vec[0],nbr[1]+vec[1]) not in self.graph[nbr]:
                      dist_src[nbr][0] = dist_src[nbr][0]"""
                      #dist_src[nbr][1] = 0
                
                  
                  #print("Nbr:",nbr,dist_src[nbr])				   
             #if flag == True:
                #dist_src[nbr][0]=dist_src[curr_exam][0]+1
                   
                
				  
                  #print("Current: ", dist_src[curr_exam],curr_exam)
                  #print("Nbr appended",dist_src[nbr],nbr)
                 
             #print("-------------------------------")	  
             to_examin.remove(curr_exam)
             examined.append(curr_exam) 

        path=[]
        curr_vertex = target
        #print("Items:")
        #for item in dist_src.items():
           #print(item)
        #print("Source: ",src, " Target: ", target)
        #print("working:",dist_src[self.found])
        while True:
             path.append(curr_vertex)
             #print(curr_vertex,dist_src[curr_vertex][2])
             direct = dir_move[dist_src[curr_vertex][2]]
             mv=(direct[0]*-1,direct[1]*-1)
             curr_vertex = (curr_vertex[0]+mv[0],curr_vertex[1]+mv[1])
             if tuple(curr_vertex) == tuple(src):
                break
        path.append(src)
        #print("this:",path)
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
			 
    def if_found(self):
        if tuple(self.found) in self.goal:
          #print("Found True:")
          return True
        else:
          #print("False")
          return False
		  
		  
    def is_optimal(self):
        self.patch_cell()
        path_closed = self.modified_djkstra2((0,0))
        path_open = self.modified_djkstra_open((0,0))
        #print("paths,",path_closed)
        #print(path_open)
        self.get_waypoints(path_open)
        open_len=len(self.waypoints)
        self.get_waypoints(path_closed)
        closed_len=len(self.waypoints)
        #print("path lengths:", open_len, closed_len)
        if open_len == closed_len:
           self.waypoints=[]
           return True
        else:
           self.waypoints=[]
           return False
		   
    def patch_cell(self):
        for i in range(self.maze_dim):
           for j in range(self.maze_dim):
              if self.visited[(i,j)] == 0:
                nbrs = self.get_neighbours((i,j))
                count = 0
                for nb in nbrs:
                  if self.visited[tuple(nb)] == 1:
                    count+=1
                  else:
                    break
                if count == len(nbrs):
                  self.visited[(i,j)] = 1