#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Empty

import os.path
from time import sleep

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.core.sprite_system import framerate_regulator
from luma.oled.device import ssd1306

import pygame
import random

pygame.font.init()

# GLOBALS VARS
s_width = 128
s_height = 64
play_width = 30  # meaning 30 // 10 = 3 width per block
play_height = 60  # meaning 60 // 20 = 3 height per block
block_size = 3

top_left_x = 1 #(s_width - play_width) // 2
top_left_y = 1 # s_height - play_height

action_list = []
start_game = []





# SHAPE FORMATS

S = [['.....',
      '.....',
      '..00.',
      '.00..',
      '.....'],
     ['.....',
      '..0..',
      '..00.',
      '...0.',
      '.....']]

Z = [['.....',
      '.....',
      '.00..',
      '..00.',
      '.....'],
     ['.....',
      '..0..',
      '.00..',
      '.0...',
      '.....']]

I = [['..0..',
      '..0..',
      '..0..',
      '..0..',
      '.....'],
     ['.....',
      '0000.',
      '.....',
      '.....',
      '.....']]

O = [['.....',
      '.....',
      '.00..',
      '.00..',
      '.....']]

J = [['.....',
      '.0...',
      '.000.',
      '.....',
      '.....'],
     ['.....',
      '..00.',
      '..0..',
      '..0..',
      '.....'],
     ['.....',
      '.....',
      '.000.',
      '...0.',
      '.....'],
     ['.....',
      '..0..',
      '..0..',
      '.00..',
      '.....']]

L = [['.....',
      '...0.',
      '.000.',
      '.....',
      '.....'],
     ['.....',
      '..0..',
      '..0..',
      '..00.',
      '.....'],
     ['.....',
      '.....',
      '.000.',
      '.0...',
      '.....'],
     ['.....',
      '.00..',
      '..0..',
      '..0..',
      '.....']]

T = [['.....',
      '..0..',
      '.000.',
      '.....',
      '.....'],
     ['.....',
      '..0..',
      '..00.',
      '..0..',
      '.....'],
     ['.....',
      '.....',
      '.000.',
      '..0..',
      '.....'],
     ['.....',
      '..0..',
      '.00..',
      '..0..',
      '.....']]

shapes = [S, Z, I, O, J, L, T]
#shape_colors = [(0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 255, 0), (255, 165, 0), (0, 0, 255), (128, 0, 128)]
shape_colors=[1,1,1,1,1,1,1]
# index 0 - 6 represent shape


class Piece(object):  # *
    def __init__(self, x, y, shape):
        self.x = x
        self.y = y
        self.shape = shape
        self.color = 1 #shape_colors[shapes.index(shape)]
        self.rotation = 0


def create_grid(locked_pos={}):  # *
	grid = [[0 for _ in range(10)] for _ in range(20)]
	for i in range(len(grid)):
		for j in range(len(grid[i])):
			if (j, i) in locked_pos:
				c = locked_pos[(j,i)]
				grid[i][j] = c
	#print grid
	return grid


def convert_shape_format(shape):
    positions = []
    format = shape.shape[shape.rotation % len(shape.shape)]

    for i, line in enumerate(format):
        row = list(line)
        for j, column in enumerate(row):
            if column == '0':
                positions.append((shape.x + j, shape.y + i))

    for i, pos in enumerate(positions):
        positions[i] = (pos[0] - 2, pos[1] - 4)

    return positions


def valid_space(shape, grid):
    accepted_pos = [[(j, i) for j in range(10) if grid[i][j] == 0] for i in range(20)]
    accepted_pos = [j for sub in accepted_pos for j in sub]

    formatted = convert_shape_format(shape)

    for pos in formatted:
        if pos not in accepted_pos:
            if pos[1] > -1:
                return False
    return True


def check_lost(positions):
    for pos in positions:
        x, y = pos
        if y < 1:
            return True

    return False


def get_shape():
    return Piece(5, 0, random.choice(shapes))
        
def clear_rows(grid, locked):

    inc = 0
    for i in range(len(grid)-1, -1, -1):
        row = grid[i]
        if 0 not in row:
            inc += 1
            ind = i
            for j in range(len(row)):
                try:
                    del locked[(j,i)]
                except:
                    continue

    if inc > 0:
        for key in sorted(list(locked), key=lambda x: x[1])[::-1]:
            x, y = key
            if y < ind:
                newKey = (x, y + inc)
                locked[newKey] = locked.pop(key)

    return inc


"""def draw_next_shape(shape, surface):
	sx = top_left_x + play_width + 50
	sy = top_left_y + play_height/2 - 100
	format = shape.shape[shape.rotation % len(shape.shape)]

	for i, line in enumerate(format):
		row = list(line)
		for j, column in enumerate(row):
			if column == '0':
				with canvas(device) as draw:
					draw.rectangle((sx+j*block_size,sy+i*block_size,block_size,block_size),fill="red")"""

def update_score(nscore):
    score = max_score()

    with open('/home/group-1/dev/catkin_ws/src/ros_tetris_oled/src/scores.txt', 'w') as f:
        if int(score) > nscore:
            f.write(str(score))
        else:
            f.write(str(nscore))


def max_score():
    with open('/home/group-1/dev/catkin_ws/src/ros_tetris_oled/src/scores.txt', 'r') as f:
        lines = f.readlines()
        score = lines[0].strip()

    return score


def draw_window(device, grid, shape, score=0, last_score = 0,level_time = 0.12):
	device.clear()
	sx = top_left_x
	sy = top_left_y
	with canvas(device) as draw:
		draw.rectangle((0,0,33,63), outline="red")
		draw.text((50,5), "Next Shape", fill="red")
		draw.text((50,20), 'Score: ' + str(score), fill="red")
		draw.text((50,35), 'High Score: ', fill="red")
		draw.text((50,50), str(last_score), fill="red")
		#draw_grid(device, grid)
		#for i in range(len(grid)):
		#    draw.line((sx,sy+i*block_size,sx+play_width,sy+i*block_size),fill="red")
		#    for j in range(len(grid[i])):
		#        draw.line((sx+j*block_size,sy,sx+j*block_size,sy+play_height),fill="red")
		
		for i in range(len(grid)):
			for j in range(len(grid[i])):
				draw.rectangle((sx+j*block_size, sy+i*block_size, sx+j*block_size+block_size, sy+i*block_size+block_size),fill=(grid[i][j]))
		
		
		sx = top_left_x + 110
		sy = top_left_y + 5
		format = shape.shape[shape.rotation % len(shape.shape)]

		for i, line in enumerate(format):
			row = list(line)
			for j, column in enumerate(row):
				if column == '0':
					draw.rectangle((sx+j*block_size,sy+i*block_size,sx+j*block_size+block_size,sy+i*block_size+block_size),fill="red")
	sleep(level_time*5)
            
        
    
    
    #pygame.display.update()


def main(device):  # *
	device.clear()

	last_score = max_score()
	locked_positions = {}

	grid = create_grid(locked_positions)

	change_piece = False
	run = True
	current_piece = get_shape()
	next_piece = get_shape()
	clock = pygame.time.Clock()
	fall_time = 0
	fall_speed = 0.27
	level_time = 0
	score = 0

	while run:
		grid = create_grid(locked_positions)
		fall_time += clock.get_rawtime()
		level_time += clock.get_rawtime()
		clock.tick()

		if level_time/1000 > 5:
			level_time = 0
			if level_time > 0.12:
				level_time -= 0.005

		if fall_time/1000 > fall_speed:
			fall_time = 0
			current_piece.y += 1
			if not(valid_space(current_piece, grid)) and current_piece.y > 0:
				current_piece.y -= 1
				change_piece = True
        
        #this is where I am going to edit to allow for ros subscribers
        # so instead of checking for events in the pygame, a buffer will be checked for 
        # numbers. Each number is associated with an action. Actions will be able to control the current piece as well as quit the game.
        
		#print action_list
        
		while action_list:
			action = action_list[0]
			action_list.pop(0)
			#print action

			if action == 0:
				run = False
			elif action == 1:
				current_piece.x -= 1
				if not(valid_space(current_piece, grid)):
					current_piece.x += 1
			elif action == 2:
				current_piece.x += 1
				if not(valid_space(current_piece, grid)):
					current_piece.x -= 1
			elif action == 3:
				current_piece.y += 1
				if not(valid_space(current_piece, grid)):
					current_piece.y -= 1
			elif action == 4:
				current_piece.rotation += 1
				if not(valid_space(current_piece, grid)):
					current_piece.rotation -= 1
        
        #for event in pygame.event.get():
        #    if event.type == pygame.QUIT:
        #        run = False
        #        device.clear()

        #    if event.type == pygame.KEYDOWN:
        #        if event.key == pygame.K_LEFT:
        #            current_piece.x -= 1
        #            if not(valid_space(current_piece, grid)):
        #                current_piece.x += 1
        #        if event.key == pygame.K_RIGHT:
        #            current_piece.x += 1
        #            if not(valid_space(current_piece, grid)):
        #                current_piece.x -= 1
        #        if event.key == pygame.K_DOWN:
        #            current_piece.y += 1
        #            if not(valid_space(current_piece, grid)):
        #                current_piece.y -= 1
        #        if event.key == pygame.K_UP:
        #            current_piece.rotation += 1
        #            if not(valid_space(current_piece, grid)):
        #                current_piece.rotation -= 1

		shape_pos = convert_shape_format(current_piece)

		for i in range(len(shape_pos)):
			x, y = shape_pos[i]
			if y > -1:
				grid[y][x] = current_piece.color

		if change_piece:
			for pos in shape_pos:
				p = (pos[0], pos[1])
				locked_positions[p] = current_piece.color
			current_piece = next_piece
			next_piece = get_shape()
			change_piece = False
			score += clear_rows(grid, locked_positions) * 10
		
		draw_window(device, grid, next_piece, score, last_score,fall_speed)
		#draw_next_shape(next_piece, device)
		#pygame.display.update()
		
		#if True:
		if check_lost(locked_positions):
			with canvas(device) as draw:
				draw.text((10,10), "You Lose!", fill="red")
				sleep(1.5)
			run = False
			update_score(score)
			
		


        


def main_menu(device,start_game):  # *
	start_game[:] = []
	run = True
	print "Hi code I am here :) "
	sleep(1)
	while run:
		with canvas(device) as draw:
			draw.rectangle(device.bounding_box, outline="black", fill="black")
			draw.text((10,10), "Press Play", fill="red")
			draw.text((15,30), "To Start", fill="red")
			

		if (start_game):
			main(device)
			start_game[:] = []
        
	device.clear()
    
# Ros subscriber
def action_callback(data):
	#print data
	action_list.append(data.data)


	
def game_start_callback(data):
	start_game.append(1);

if __name__ == '__main__':
	try:
		serial = i2c(port=1, address=0x3C)
		device = ssd1306(serial, rotate=0)
		
		rospy.init_node('ros_tetris_node')
		rospy.Subscriber("ros_tetris_game_start", Empty, game_start_callback)
		rospy.Subscriber("ros_tetris_listener", Int32, action_callback)
		
		action_list[:] = []
		
		device.clear()

		if device.width < 96 or device.height < 64:
			raise ValueError("Unsupported mode: {0}x{1}".format(device.width, device.height))
			
		regulator = framerate_regulator()
		main_menu(device,start_game)
	except rospy.ROSInterruptException:
		pass
	
