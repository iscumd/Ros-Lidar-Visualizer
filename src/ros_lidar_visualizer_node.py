#!/usr/bin/env python
import pygame
import rospy
import math
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries


BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
DEEP_RED = (155,0,0)
YELLOW = (255,255,0)
PURPLE = (59,11,48)

screen_width = 900
screen_height = 1000

X_SCALE = 1.5
Y_SCALE = 0.6


pygame.init()
size = (screen_width, screen_height)
course_size = [400,1500]
screen = pygame.display.set_mode(size)

obstacles = [[0, 0], [0, 0]]
robot_pose = [0,0,0]
count = 0.01
origin_offset = [200,course_size[1] - 250]
robot_pose[0] = robot_pose[0] + origin_offset[0]
robot_pose[1] = robot_pose[1] + origin_offset[1]

def draw_yeti(x,y,rot): #input scaled x and y
    outer_dim = [[0,0],[0.45*100*X_SCALE,0],[0.45*100*X_SCALE,0.75*100*Y_SCALE],[0,0.75*100*Y_SCALE]]
    rotated_outer_dim = []
    for i in range(0,len(outer_dim)):
        rotated_outer_dim.append(rotate_around_point(0.225*100*X_SCALE,0.7*100*Y_SCALE,rot,outer_dim[i][0] + x,outer_dim[i][1] + y ))
    print rotated_outer_dim
    pygame.draw.polygon(screen,BLUE,rotated_outer_dim,0)

def rotate_around_point(center_x,center_y,angle,x,y):
    s = math.sin(angle)
    c = math.cos(angle)
    x = x - center_x
    y = y - center_y
    new_x = x * c - y * s
    new_y = x * s + y * c
    x = new_x + center_x
    y = new_y + center_y
    return [x,y]

def midpoint(p1, p2):
    return [(p1[0]+p2[0])/2, (p1[1]+p2[1])/2]

def pscale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def distance(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def polar_to_cartesian(theta, rot):
    x = math.cos(theta) * rot
    y = math.sin(theta) * rot
    return [x,y]

def text_to_screen(screen, text, x, y, size = 20, color = (000, 000, 000)):
    font_type = pygame.font.match_font("roboto-black", "Arial", "sansserif")#'./Roboto-Black.ttf'
    try:

        text = str(text)
        font = pygame.font.Font(font_type, size)
        text = font.render(text, True, color)
        screen.blit(text, (x, y))

    except Exception, e:
        print 'Font Error, saw it coming'
        raise e

def refresh_screen():
    pygame.display.flip()
    screen.fill(DEEP_RED)
    pygame.draw.rect(screen, BLUE, [0,0,X_SCALE*course_size[0],Y_SCALE*course_size[1]], 2)
    pygame.draw.rect(screen, WHITE, [X_SCALE*150, Y_SCALE*200, X_SCALE * 100, Y_SCALE * 1000], 0)

def laserCallback(scan):
    #print "got laser"
    #lidar_max_range = data.range_max
    global count
    start = scan.angle_min

    for i in range(0, len(scan.ranges)):
        point = polar_to_cartesian(start + scan.angle_increment * i, scan.ranges[i])
        final_x = ((point[0]*100) + robot_pose[0])*X_SCALE
        final_y = (robot_pose[1] - (point[1]*100))*Y_SCALE
        pygame.draw.circle(screen, GREEN, (int(final_x), int(final_y)), 3)
    #draw_yeti(robot_pose[0] * X_SCALE, robot_pose[1]*Y_SCALE, count*math.pi/20)

    count = count + 1
    refresh_screen()

def obstacleCallback(obstacles):
    obstacles = obstacles.poses
    for i in range(0,len(obstacles)):
        obs_x = (obstacles[i].position.x * 100) * X_SCALE
        obs_y = (obstacles[i].position.y * 100) * Y_SCALE
        pygame.draw.circle(screen, YELLOW, (int(final_x), int(final_y)), 3)
    refresh_screen()

def poseCallback(pose):
    print "got pose"

def main():
    rospy.init_node('visualizer', anonymous=True)

    pygame.display.set_caption("Localization visualizer")

    refresh_screen()


    rospy.Subscriber("scan", sensors.LaserScan, laserCallback)
    rospy.Subscriber("obstacles", geometries.PoseArray, obstacleCallback)
    rospy.Subscriber("yeti/pose",geometries.Pose2D,poseCallback)
    rospy.spin()

if __name__ == '__main__':
    main()