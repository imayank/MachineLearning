from maze import Maze
import turtle
import sys

if __name__ == '__main__':
    '''
    This function uses Python's turtle library to draw a picture of the maze
    given as an argument when running the script.
    '''

    # Create a maze based on input argument on command line.
    testmaze = Maze( str(sys.argv[1]) )

    # Intialize the window and drawing turtle.
    window = turtle.Screen()
    wally = turtle.Turtle()
    wally.speed(0)
    wally.hideturtle()
    wally.penup()

    # maze centered on (0,0), squares are 20 units in length.
    sq_size = 30
    #origin=0
    origin = testmaze.dim * sq_size / -2
	

    # iterate through squares one by one to decide where to draw walls
    for x in range(testmaze.dim):
        for y in range(testmaze.dim):
			if not testmaze.is_permissible([x,y], 'up'):
				wally.goto(origin + sq_size * x, origin + sq_size * (y+1))
				wally.setheading(0)
				wally.pendown()
				wally.forward(sq_size)
				wally.penup()

			if not testmaze.is_permissible([x,y], 'right'):
				wally.goto(origin + sq_size * (x+1), origin + sq_size * y)
				wally.setheading(90)
				wally.pendown()
				wally.forward(sq_size)
				wally.penup()

            # only check bottom wall if on lowest row
			if y == 0 and not testmaze.is_permissible([x,y], 'down'):
				wally.goto(origin + sq_size * x, origin)
				wally.setheading(0)
				wally.pendown()
				wally.forward(sq_size)
				wally.penup()

            # only check left wall if on leftmost column
			if x == 0 and not testmaze.is_permissible([x,y], 'left'):
				wally.goto(origin, origin + sq_size * y)
				wally.setheading(90)
				wally.pendown()
				wally.forward(sq_size)
				wally.penup()
			
			wt = 0
			if testmaze.is_permissible([x,y], 'up'):
				wt+=1
			if testmaze.is_permissible([x,y], 'right'):
				wt+=2
			if testmaze.is_permissible([x,y], 'down'):
				wt+=4
			if testmaze.is_permissible([x,y], 'left'):
				wt+=8
			
			wally.goto(origin + sq_size * (x+0.3), origin + sq_size * (y+0.2))
			wally.pendown()
			wally.write(wt)
			wally.penup()
    window.exitonclick()