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

screen_width = 1000
screen_height = 900

pygame.init()
size = (screen_width, screen_height)
screen = pygame.display.set_mode(size)

obstacles = [[0, 0], [0, 0]]
lidar_max_range = 20


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


def laserCallback(data):

    global lidar_max_range
    lidar_max_range = data.range_max
    start = data.angle_min

    pygame.draw.line(screen, BLUE, (screen_width/2, 0), (screen_width/2, screen_height), 1)
    pygame.draw.line(screen, BLUE, (0, screen_height-100), (screen_width, screen_height-100), 1)

    for i in range(0,len(data.ranges)):
        point = polar_to_cartesian(start + data.angle_increment*i, data.ranges[i])
        scaled_x = int(pscale(point[0],0,data.range_max,0,screen_width - 50))
        scaled_y = int(pscale(point[1], 0, data.range_max, 0, screen_height - 50))
        translated_x = scaled_x + screen_width/2
        translated_y = screen_height - scaled_y - 100
        final_x = translated_x
        final_y = translated_y
        pygame.draw.circle(screen, GREEN, (final_x, final_y), 3)

    for i in range(0,len(obstacles)):
        scaled_obs_x = int(pscale(obstacles[i][0],0,data.range_max,0,screen_width-50))
        scaled_obs_y = int(pscale(obstacles[i][1],0,data.range_max,0,screen_height-50))
        translated_obs_x = scaled_obs_x + screen_width/2
        translated_obs_y = screen_height - scaled_obs_y - 100
        pygame.draw.circle(screen, WHITE, (translated_obs_x, translated_obs_y), 10)
        pygame.draw.line(screen,PURPLE,(screen_width/2,screen_height-100),(translated_obs_x,translated_obs_y),4)
        dist = distance(0, 0, obstacles[i][0], obstacles[i][1])
        mid = midpoint([screen_width / 2, screen_height - 100], [translated_obs_x, translated_obs_y])
        mid_x = mid[0] - 10
        mid_y = mid[1]
        text_to_screen(screen, str(dist)[:5] + 'm', mid_x, mid_y)

    refresh_screen()


def obstacle1Callback(data):
    obstacles[0][0] = data.x
    obstacles[0][1] = data.y

    #pygame.display.flip()
    #screen.fill(DEEP_RED)
    #pygame.draw.rect(screen, YELLOW, [screen_width / 2 - 25 / 2, 7 * screen_height / 8 + 10, 25, 50], 2)


def obstacle2Callback(data):
    obstacles[1][0] = data.x
    obstacles[1][1] = data.y

    #pygame.display.flip()
    #screen.fill(DEEP_RED)
    #pygame.draw.rect(screen, YELLOW, [screen_width / 2 - 25 / 2, 7 * screen_height / 8 + 10, 25, 50], 2)


def pointCloudCallback(data):
    # print "got point cloud"
    for i in range(0, len(data.points)):
        point = []
        point.append(data.points[i].x)
        point.append(data.points[i].y)
        scaled_x = int(pscale(point[0], 0, lidar_max_range, 0, screen_width - 50))
        scaled_y = int(pscale(point[1], 0, lidar_max_range, 0, screen_height - 50))
        translated_x = scaled_x + screen_width / 2
        translated_y = screen_height - scaled_y - 100
        final_x = translated_x
        final_y = translated_y
        pygame.draw.circle(screen, BLUE, (final_x, final_y), 3)

def refresh_screen():
    pygame.display.flip()
    screen.fill(DEEP_RED)
    pygame.draw.rect(screen, YELLOW, [screen_width / 2 - 25 / 2, 7 * screen_height / 8 + 10, 25, 50], 2)
    pygame.draw.line(screen, BLUE, (screen_width / 2, 0), (screen_width / 2, screen_height), 1)
    pygame.draw.line(screen, BLUE, (0, screen_height - 100), (screen_width, screen_height - 100), 1)


def listener():
    rospy.init_node('visualizer', anonymous=True)

    pygame.display.set_caption("Localization visualizer")

    refresh_screen()


    rospy.Subscriber("scan", sensors.LaserScan, laserCallback)
    rospy.Subscriber("scan_filtered", sensors.PointCloud, pointCloudCallback)
    rospy.Subscriber("obstacle1", geometries.Pose2D, obstacle1Callback)
    rospy.Subscriber("obstacle2", geometries.Pose2D, obstacle2Callback)
    rospy.spin()

if __name__ == '__main__':
    listener()