from controller import Robot

bot = Robot()

timestep = 64

# getting devices
cam = bot.getDevice('camera')
left_wheel = bot.getDevice('left_front_wheel')
right_wheel = bot.getDevice('right_front_wheel')
l_steer = bot.getDevice('left_steer')
r_steer = bot.getDevice('right_steer')
lidar = bot.getDevice('Sick LMS 291')

# initialisations
cam.enable(timestep)
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
l_steer.setPosition(0)
r_steer.setPosition(0)
left_wheel.setVelocity(0)
right_wheel.setVelocity(0)
lidar.enable(timestep)
lidar.enablePointCloud()

def wait(time_steps):
    time_counter = 0 
    while bot.step(timestep ) != -1:
        if time_counter >= time_steps:
            break
        time_counter += 1
        
            

# main loop
while bot.step(timestep) != -1:
    val = lidar.getRangeImage()
    print(val)
    affected_rays = val[85:96]
    obstacle_detected = any([value < 5 for value in affected_rays])
    print(obstacle_detected)
    # image data
    img = cam.getImage()
    image_width = cam.getWidth()
    image_height = cam.getHeight()
    
    # processing image, method 1
    # getting average position of yellow pixels
    # getting total yellow pixels
    
    x_yellow = []
    for x in range(0, image_width):
        for y in range(0, image_height):
            red_val = cam.imageGetRed(img, image_width, x, y)
            green_val = cam.imageGetGreen(img, image_width, x, y)
            blue_val = cam.imageGetBlue(img, image_width, x, y)
            if red_val > 190 and green_val > 180 and blue_val > 90:
                x_yellow.append(x)
    
    # finding average of yellow pixels
    if x_yellow: # if there are any yellow pixels
        x_total = 0
        for x in x_yellow:
            x_total = x_total + x
        x_average = x_total / len(x_yellow)
    
    # rotating steering angle so that yellow lane remains in the center
    x_center = image_width / 2
    
    if obstacle_detected:
        l_steer.setPosition(-0.7)
        r_steer.setPosition(-0.7)
        wait(10)
        l_steer.setPosition(0)
        r_steer.setPosition(0)
        wait(20)
        l_steer.setPosition(0.7)
        r_steer.setPosition(0.7)
        wait(10)
    else :
        if x_average < x_center: # max pixels are on the left, take a left turn
            l_steer.setPosition(-0.1)
            r_steer.setPosition(-0.1)
        
        elif x_average > x_center: # max pixels are on the right, take right turn
            l_steer.setPosition(0.1)
            r_steer.setPosition(0.1)
        
    # move forward
    left_wheel.setVelocity(10)
    right_wheel.setVelocity(10)
