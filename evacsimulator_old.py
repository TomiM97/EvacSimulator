import numpy as np
import pygame
import pyvisgraph as vg

# Simulation parameters
time_step = 0.2
num_steps = 200
number_of_agents = 10
agent_radius = 5 # Unit is 100mm

# Define a target for agents to move towards
target_x, target_y = 50, 50

# top left corner is (0.0, 0.0) and down right corner is (SCREEN_WIDTH, SCREEN_HEIGHT)
polys = [[vg.Point(100.0,500.0), vg.Point(800.0,500.0), vg.Point(800,300.0), vg.Point(790,300.0),
          vg.Point(790,490.0), vg.Point(110,490.0), vg.Point(110,110.0), vg.Point(790,110.0), 
          vg.Point(790,280.0), vg.Point(800,280.0), vg.Point(800,100.0), vg.Point(100,100.0)],
          [vg.Point(200.0,400.0), vg.Point(700.0,400.0), vg.Point(700,200.0), vg.Point(200,200.0)]]

# initialization for visualization
pygame.init()
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

# Colour initialization for pygame
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Agent:
    def __init__(self, x, y, vx, vy, speed, radius=agent_radius):
        self.x = x              # Current x position
        self.y = y              # Current y position
        self.vx = vx            # Velocity in x direction
        self.vy = vy            # Velocity in y direction
        self.speed = speed      # Current speed
        self.radius = radius    # Radius
        self.path = []          # Store the computed path
        self.path_index = 0     # Index for the next point in the path

    def update(self, dt):
        # Update position based on velocity and time step dt
        self.x += self.vx * dt
        self.y += self.vy * dt

    def checkClosestRoute(self, polys, radius):
        # Takes the agent radius into consideration
        inflated_obstacles = [inflatePolygon(polygon, radius + 5) for polygon in polys]

        g = vg.VisGraph()
        g.build(inflated_obstacles)

        start = vg.Point(self.x, self.y)
        goal = vg.Point(target_x, target_y)

        self.path = g.shortest_path(start, goal)
        self.path_index = 0


    def moveAlongPath(self, dt, collision):
        # Move the agent along the stored path
        if self.path_index < len(self.path):
            # Target the next point on the path
            target_point = self.path[self.path_index]
            target_x, target_y = target_point.x, target_point.y

            # Move towards the next point on the path
            self.moveTowards(target_x, target_y, dt, collision)

            # Check if the agent is close enough to the next point
            if np.sqrt((self.x - target_x) ** 2 + (self.y - target_y) ** 2) < 1:
                self.path_index += 1  # Move to the next point on the path
    
    def moveTowards(self, target_x, target_y, dt):
        # Calculate direction towards target
        direction_x = target_x - self.x
        direction_y = target_y - self.y
        distance = np.sqrt(direction_x**2 + direction_y**2)
        if collision:
            direction_x = 0
            direction_y = 0
                
        # Normalize direction and update velocity based on speed
        if distance > 0:
            self.vx = (direction_x / distance) * self.speed
            self.vy = (direction_y / distance) * self.speed

        
        # Update the position after applying the movement
        self.update(dt)  
    
    def draw(self, screen):
        # Draw the agent as a circle
        pygame.draw.circle(screen, RED, (int(self.x), int(self.y)), agent_radius)


    ## Collision detection ##

    def isCollidingWithPolygon(self, polys):
        for poly in polys:
            for i in range(len(poly)):
                p1 = poly[i]
                p2 = poly[(i + 1) % len(poly)]
                # Check if agent's current or next position collides with any polygon edge
                if line_intersects_circle((p1.x, p1.y), (p2.x, p2.y), (self.x, self.y), self.radius):
                    return True
        return False

    def isCollidingWithOtherAgents(self, agents):
        for other_agent in agents:
            if other_agent != self:
                dist = np.sqrt((self.x - other_agent.x)**2 + (self.y - other_agent.y)**2)
                if dist < (self.radius + other_agent.radius):
                    return True
        return False

def line_intersects_circle(p1, p2, center, radius):
    # Helper function to check if line segment intersects with circle (agent)
    closest_point = closest_point_on_line(p1, p2, center)
    dist = np.sqrt((closest_point[0] - center[0])**2 + (closest_point[1] - center[1])**2)
    return dist <= radius

def closest_point_on_line(p1, p2, point):
    # Find the closest point on a line segment to a point (center of agent)
    line_vec = np.array(p2) - np.array(p1)
    point_vec = np.array(point) - np.array(p1)
    line_len = np.dot(line_vec, line_vec)
    t = max(0, min(1, np.dot(point_vec, line_vec) / line_len))
    closest = np.array(p1) + t * line_vec
    return closest

## For inflating the polygons for pathfinding ##

def normalize(vector):
    length = np.linalg.norm(vector)
    return vector / length if length != 0 else vector

def inflatePolygon(polygon, radius):
    inflated_polygon = []
    num_vertices = len(polygon)
    previous_normal_vector = np.array([])
    
    for i in range(num_vertices):
        # Current edge
        p1 = np.array([polygon[i].x, polygon[i].y])
        p2 = np.array([polygon[(i + 1) % num_vertices].x, polygon[(i + 1) % num_vertices].y])
        
        # Edge vector and perpendicular normal vector
        edge_vector = p2 - p1
        normal_vector = np.array([-edge_vector[1], edge_vector[0]])  # Perpendicular
        normal_vector = normalize(normal_vector) * radius

        if previous_normal_vector.size != 0:
            if is_inner_corner(previous_normal_vector, normal_vector):
                inflated_p2 = p2 + normal_vector
                inflated_polygon.append(vg.Point(inflated_p2[0], inflated_p2[1]))
            else:
                # Inflate the two points of the edge
                inflated_p1 = p1 + normal_vector
                inflated_p2 = p2 + normal_vector
                inflated_polygon.append(vg.Point(inflated_p1[0], inflated_p1[1]))
                inflated_polygon.append(vg.Point(inflated_p2[0], inflated_p2[1]))
            previous_normal_vector = normal_vector
        else:
            previous_normal_vector = normal_vector
            inflated_p1 = p1 + normal_vector
            inflated_p2 = p2 + normal_vector
            inflated_polygon.append(vg.Point(inflated_p1[0], inflated_p1[1]))
            inflated_polygon.append(vg.Point(inflated_p2[0], inflated_p2[1]))
    
    return inflated_polygon

def is_inner_corner(previous_normal_vector, current_normal_vector):
    # Step 1: Compute the dot product
    dot_product = previous_normal_vector[0] * current_normal_vector[0] + previous_normal_vector[1] * current_normal_vector[1]
    # Step 2: Compute the 2D cross product
    cross_product = previous_normal_vector[0] * current_normal_vector[1] - previous_normal_vector[1] * current_normal_vector[0]
    
    # Step 3: Check conditions for inner corner
    # Inner corner happens when:
    # - Dot product is negative (angle >= 90 degrees)
    # - Cross product is positive
    if dot_product <= 0 and cross_product > 0:
        return True  # It's an inner corner
    else:
        return False  # Not an inner corner
    
# Function to place agents in valid locations
def spawn_agents_valid_positions():
    agents = []
    for _ in range(number_of_agents):
        valid_position = False
        while not valid_position:
            new_agent = Agent(np.random.uniform(0, SCREEN_WIDTH), np.random.uniform(0, SCREEN_HEIGHT), 0, 0, 5)
            # Check if the new agent is not inside any polygon and not overlapping with existing agents
            if not new_agent.isCollidingWithPolygon(polys) and not any(new_agent.isCollidingWithOtherAgents([agent]) for agent in agents):
                agents.append(new_agent)
                valid_position = True
    return agents
    
## Main ##

def main():
    run = True
    agents = spawn_agents_valid_positions()
    clock = pygame.time.Clock()

    # Calculate initial paths
    for agent in agents:
        agent.checkClosestRoute(polys, radius=agent_radius)

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        
        # Clear the screen
        screen.fill(WHITE)
        
        # Update each agent's position
        for agent in agents:
            if not agent.isCollidingWithPolygon(polys) and not agent.isCollidingWithOtherAgents(agents):
                agent.moveAlongPath(time_step, collision=False)
                agent.draw(screen)
            else:
                agent.moveAlongPath(-time_step, collision=True)
                agent.draw(screen)

        # Draw obstacles
        for poly in polys:
            polygon_points = [(int(p.x), int(p.y)) for p in poly]
            pygame.draw.polygon(screen, BLACK, polygon_points, 0)
   
        pygame.display.update()

        # Cap the frame rate to 60 FPS
        clock.tick(60)

    pygame.quit()

main()