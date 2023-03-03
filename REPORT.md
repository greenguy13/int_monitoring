# COSC.269-FA22, PA1: Simple Shapes
## Author: Amel Docena


## Method Description
The program starts with configuring the parameters for the simulation, which include the linear velocity and the angular velocity. Furthermore,
we set the parameters for the traversal of the polygon-shaped route: these would be the number of sides of the polygon, the side length, and the orientation 
whether clockwise or counter-clockwise. We pass these arguments into the `PolygonMotion` class where we subscribe to the `laser` and `odom` topics and publish 
velocity commands.  

Now inside this class, we first instantiate a node for the operation. We then employ the necessary methods that build up the algorithm of the traversal. 
The idea of the algorithm is that we start from the initial position (0, 0) as starting coordinates, then traverse the first side of the polygon. When we reach
its length, we rotate in place to form one vertex of the polygon. We then journey forward by this angle traversing the given side length until we traverse it fully and
rotate again in place to form another vertex of the polygon. We keep doing this until we have completed all sides, and theoretically, come back to initial position (the last vertex),
thereby traversing the entire shape of the (closed) polygon.  

To formulate the details of the algorithm, we define methods `move_forward` and `rotate_in_place`. The method `move_forward` tells the robot to traverse the distance for the duration 
given its velocity. The method `rotate_in_place` tells the robot to rotate in place by the computed rotation, rotating by the given orientation whether clockwise or counter-clockwise. 
Note that the interior angles, `a`, of a polygon can be computed as`a = ((n-2)*pi)/n`, 
where `n` is the number of sides and `pi` is the angle 180 degrees in radians. Since we are rotating from a forward direction, we subtract the interior angle `a` from `pi` (or 180 degrees), which
represents the forward (or straight) direction. The computed rotation is thus `r = pi - a`. We now have the components for the algorithm:

`For n in range(number of sides):`  
`> move forward given the side length`  
`> rotate in place by the computed angle given the orientation (clockwise or counter-clockwise)`




## Evaluation and Discussion
We simulate three polygon traversals: a triangle with side length of units oriented clockwise; a square with side length of 3 units oriented counter-clockwise; 
and a pentagon with side length 5 units oriented clockwise. The respective videos are stored inside the `videos` folder. The result of the simulation is as expected from the algorithm. The robot traverses the first side of the polygon, rotating approximately by the computed angle as it reaches the vertex given the orientation; 
this behavior is repeated until the entire traversal is accomplished and the robot is back (somewhere around) where it started. 

Note, however, that the actual vertices of the robot are not what is expected (if we solve by pen and paper). (We store screenshot of the actual vertices of each of the polygon traversals inside the `videos` folder.)
If we take out one result, say the triangle-shaped traversal, there are some slight deviations but still within reasonable neighborhood of the expected vertices. Starting from the initial coordinates (0, 0), we expect the coordinates of the first vertex after the robot traverses the first edge to be (3, 0), and that, the last vertex
it should reach at should be the initial coordinates (0, 0), (i.e., back from where it started). The actual vertices, however, deviate from these values. For the first vertex, the coordinates are (3.00934713117, -0.0014236144101), while the coordinates for the last vertex are (0.44648782635, 0.458759647894). We can compute for the error for each vertex by computing the distance between the expected and actual vertices, (which are points); and add them up if we want the total.
We no longer implemented this part due to lack of time to refresh on geometry/trigonometry concepts to compute for (expected) vertices of a polygon given side length and interior angle. Nevertheless, we make this point that in actual scenarios there may be inaccuracies due to mechanical issues (or "givens"), like motor of the robot, friction between the wheels and the floor, physics, hardware and computing resources. And that, these deviations should always be included or
taken into account when planning for an operation or relying on/understanding published messages from the robot.