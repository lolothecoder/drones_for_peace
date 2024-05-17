# READ ME
## What works so far
- You can send the robot a sequence of points for it to follow in the sequence array. The format of the points are (x, y, z). The robot is currently programmed to do a 50 cm square at a height of 40 cm. 

- Placing your hand over the robots makes it land.

- Dijkstra algorithm works in simulation with visualisation but has not been tested on the robot. Chosen path is in blue. It has not yet been added to the main code.

# Next Steps

## COMMON  

### Landing
The robot needs to be able to land when he sees the landing pad. It needs to make sure the landing pad is always under him while he's landing

### Search algorithm
Once the robot reaches the endzone, he needs to search for the landing pad. I think this should be easy enough you should just need to give it a list of points to go to.

## GLOBAL

### Dijkstra
The map needs to remove a node once a range sensor has detected that there is an obstacle there. The remove function exists it just need to be activated when the range sensor sees it. The position of the robot needs to also be on the map. Once the robot sees an obstacle it recalculates the best path

## LOCAL
### Obstacle avoidance
Robot needs to avoid an obstacle by going around it. Perhaps the function send_hover_point(vx, vy, yawRate, height) is more interesting than send_position_point(x, y, z, yaw) in this context since we would probably
be more interested in controlling the speeds? Not sure tho

