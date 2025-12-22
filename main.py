import pygame
import heapq

# Grid and cell dimensions
GRID_WIDTH = 20
GRID_HEIGHT = 15
CELL_SIZE = 40
MARGIN = 1

# Simulation costs
COST_MOVE_ENERGY = 1.0  # Energy used per block moved
COST_MOVE_TIME = 1.0  # Seconds taken per block moved
COST_PICK_ENERGY = 5.0  # Energy used to lift an item
COST_PICK_TIME = 3.0  # Seconds taken to lift an item

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)  # Shelves
BLUE = (0, 120, 255)  # Robot
RED = (255, 50, 50)  # Targets/Packages
YELLOW = (255, 200, 0)  # Highlight/Package Collected
GREEN = (50, 200, 50)  # Home/Success
LIGHT_BLUE = (173, 216, 230)  # Path of Robot
OVERLAY_COLOR = (0, 0, 0, 180)  # Semi-transparent black

# Animation speed
FPS = 3


class Warehouse:
    """
    A class to define a Warehouse object as a 2D grid
    """

    def __init__(self, width, height):
        """
        Warehouse class constructor

        :param width: Width of the warehouse
        :param height: Height of the warehouse
        """
        self.width = width
        self.height = height
        # construct 2D list grid
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        # set coordinates of home position on the grid
        self.home = (0, 0)

    def is_walkable(self, r, c):
        """
        Function to check if the coordinates lie within the bounds of the warehouse and are not occupied by an obstacle.

        :param r: row position
        :param c: column position
        """
        if 0 <= r < self.height and 0 <= c < self.width:
            return self.grid[r][c] != 1
        return False

    def toggle_obstacle(self, r, c):
        """
        Toggle the state of a coordinate on the grid between `obstacle` and `empty`.

        :param r: row position
        :param c: column position
        """
        if (r, c) != self.home:
            self.grid[r][c] = 1 if self.grid[r][c] == 0 else 0


# Pathfinding class, based on A* search
class PathFinder:
    def __init__(self, warehouse):
        self.wh = warehouse

    # Manhattan distance for heuristic of grid based path finding
    # |x_1 - x_2| + |y_1 - y_2|
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_path(self, start, goal):
        if not self.wh.is_walkable(*goal):
            return None
        frontier = [] # a list to store nodes we need to explore
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break

            r, c = current
            neighbors = [(r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)]
            for next_node in neighbors:
                if self.wh.is_walkable(*next_node):
                    new_cost = cost_so_far[current] + 1
                    if (
                        next_node not in cost_so_far
                        or new_cost < cost_so_far[next_node]
                    ):
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(goal, next_node)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current

        if goal not in came_from:
            return None
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return path


class Simulation:
    # handles graphics , user input and game rules
    """
    Simulation class for UI container and simulation logic and parameters.
    """

    def __init__(self):
        pygame.init()  # intialize pygame
        self.width = GRID_WIDTH * (CELL_SIZE + MARGIN) + MARGIN
        self.height = (
            GRID_HEIGHT * (CELL_SIZE + MARGIN) + MARGIN + 100
        )  # Extra UI space for controls
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Warehouse Bot: Cost & Tradeoff Simulator")
        self.clock = pygame.time.Clock()

        # Fonts
        self.font_ui = pygame.font.SysFont("Arial", 18)
        self.font_summary = pygame.font.SysFont("Arial", 24, bold=True)
        self.font_large = pygame.font.SysFont("Arial", 32, bold=True)

        # Warehouse object and Pathfinder object
        self.wh = Warehouse(GRID_WIDTH, GRID_HEIGHT)
        self.finder = PathFinder(self.wh)

        # UI Buttons
        self.btn_single = pygame.Rect(20, self.height - 80, 200, 40)
        self.btn_batch = pygame.Rect(240, self.height - 80, 200, 40)
        self.btn_run = pygame.Rect(self.width - 160, self.height - 80, 140, 40)

        # Clear grid and setup fresh simulation session
        self.reset_simulation()

    def reset_simulation(self):
        """
        Resets the state of the robot and all placed packages
        """
        self.robot_pos = (0, 0)
        self.orders = []
        self.picked_orders = []
        self.mode = "SINGLE_RETURN"
        self.state = "IDLE"  # IDLE, MOVING, FINISHED
        self.target_queue = []
        self.current_path = []

        # Cost Metrics
        self.total_energy = 0.0
        self.total_time = 0.0

    def clear_obstacles(self):
        """
        Clears all obstacles and resets the simulation
        """
        for row in self.wh.grid:
            row[:] = [0] * len(row)
        self.reset_simulation()

    def optimize_batch_route(self, start, orders):
        """
        Nearest Neighbor Logic (Travelling Salesman Problem)
        """
        # shallow copy orders list
        unvisited = orders[:]
        current = start
        route = []
        while unvisited:
            best_dist = float("inf")  # set to infinity
            best_node = None
            for node in unvisited:
                if node is not None:
                    dist = abs(current[0] - node[0]) + abs(current[1] - node[1])
                    if dist < best_dist:
                        best_dist = dist
                        best_node = node
            if best_node is not None:
                route.append(best_node)
                unvisited.remove(best_node)
                current = best_node
            else:
                break
        return route

    def start_mission(self): # functions runs when "run mission" is clicked
        if not self.orders:  # if no orders, do nothing and return
            return
        self.state = "MOVING"
        self.total_energy = 0
        self.total_time = 0
        self.picked_orders = []
        self.robot_pos = (0, 0)
        self.target_queue = []

        # Detemine mode of operation based on user selection and construct travel plan
        if self.mode == "SINGLE_RETURN":
            # Naive approach: Go to an item, collect it, go home, repeat.
            for order in self.orders:
                self.target_queue.append(order)  # go to order location
                self.target_queue.append((0, 0))  # go to home location

        elif self.mode == "BATCH_TSP":
            # Batch approach: Determine optimized path, collect orders along it, then return home.
            optimized_orders = self.optimize_batch_route((0, 0), self.orders)
            for order in optimized_orders:  # get all orders
                self.target_queue.append(order)
            self.target_queue.append((0, 0))  # then go home

        # Get next path segment
        if self.target_queue:
            path = self.finder.get_path(self.robot_pos, self.target_queue[0])
            if path is None:
                self.state = "FINISHED"  # Blocked at start
            else:
                self.current_path = path

    def draw_summary_popup(self):
        """
        Draw the summary popup at the end of the simulation
        """
        # Create a semi-transparent overlay
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill(OVERLAY_COLOR)
        self.screen.blit(overlay, (0, 0))

        # Draw summary box
        box_w, box_h = 400, 300
        box_x = (self.width - box_w) // 2
        box_y = (self.height - box_h) // 2
        pygame.draw.rect(self.screen, WHITE, (box_x, box_y, box_w, box_h))
        pygame.draw.rect(self.screen, BLUE, (box_x, box_y, box_w, box_h), 3)  # Border

        # Text content
        title = self.font_large.render("MISSION SUMMARY", True, BLUE)
        mode_txt = self.font_summary.render(f"Strategy: {self.mode}", True, BLACK)
        time_txt = self.font_summary.render(
            f"Total Time:   {self.total_time:.1f} sec", True, BLACK
        )
        enrg_txt = self.font_summary.render(
            f"Total Energy: {self.total_energy:.1f} units", True, BLACK
        )
        eff_txt = self.font_summary.render(
            f"Orders Picked: {len(self.picked_orders)}", True, GREEN
        )

        hint_txt = self.font_ui.render("Press 'R' to Reset or 'C' to Clear", True, GRAY)

        # Centering blit text
        self.screen.blit(title, (box_x + 50, box_y + 20))
        self.screen.blit(mode_txt, (box_x + 40, box_y + 80))
        self.screen.blit(time_txt, (box_x + 40, box_y + 120))
        self.screen.blit(enrg_txt, (box_x + 40, box_y + 160))
        self.screen.blit(eff_txt, (box_x + 40, box_y + 200))
        self.screen.blit(hint_txt, (box_x + 70, box_y + 260))

    def run(self):
        running = True
        while running: # infinite loop that keeps window open.
            # Handling interaction events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                # Listening for reset key press (R)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:  # Reset Key
                        self.reset_simulation()

                # Listening for clear key press (C)
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c:  # Clear Key
                        self.clear_obstacles()

                # Listening for mouse clicks
                if event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()

                    # Button Clicks
                    if self.state == "IDLE":
                        if self.btn_single.collidepoint((mx, my)):
                            self.mode = "SINGLE_RETURN"
                        elif self.btn_batch.collidepoint((mx, my)):
                            self.mode = "BATCH_TSP"
                        elif self.btn_run.collidepoint((mx, my)):
                            self.start_mission()

                        # Grid Clicks
                        c = mx // (CELL_SIZE + MARGIN)
                        r = my // (CELL_SIZE + MARGIN)
                        if 0 <= r < GRID_HEIGHT and 0 <= c < GRID_WIDTH:
                            if event.button == 1:  # Left click
                                if (
                                    self.wh.is_walkable(r, c)
                                    and (r, c) != (0, 0)
                                    and (r, c) not in self.orders
                                ):
                                    self.orders.append((r, c))
                            elif (
                                event.button == 3 and (r, c) not in self.orders
                            ):  # Right click
                                self.wh.toggle_obstacle(r, c)

            # Running simulation
            if self.state == "MOVING": # only runs if mission has started
                if self.current_path:
                    self.robot_pos = self.current_path.pop(0) # moves robot one step forward
                    # Simulate movement energy and time costs
                    self.total_energy += COST_MOVE_ENERGY
                    self.total_time += COST_MOVE_TIME

                    current_target = self.target_queue[0]
                    if self.robot_pos == current_target: # ckecks if we arrived at red square
                        if current_target != (0, 0):
                            # Pick item and simulate picking energy and time cost
                            self.picked_orders.append(current_target) # marks order as done (turns yellow)
                            self.total_energy += COST_PICK_ENERGY
                            self.total_time += COST_PICK_TIME

                        self.target_queue.pop(0)

                        # If more targets exist, get path to next target in queue
                        if self.target_queue:
                            next_target = self.target_queue[0]
                            path = self.finder.get_path(self.robot_pos, next_target)
                            if path:
                                self.current_path = path
                            else:
                                self.state = "FINISHED"  # Blocked
                        else:
                            self.state = "FINISHED"  # Done

            # Draw black background
            self.screen.fill(BLACK)

            # Draw grid
            for r in range(GRID_HEIGHT):
                for c in range(GRID_WIDTH):
                    color = WHITE
                    if self.wh.grid[r][c] == 1:  # obstacles
                        color = GRAY
                    elif (r, c) == (0, 0):  # home
                        color = GREEN
                    if (r, c) in self.orders:  # orders
                        color = RED if (r, c) not in self.picked_orders else YELLOW

                    # Highlight current path
                    if (r, c) in self.current_path:
                        pygame.draw.rect(
                            self.screen,
                            LIGHT_BLUE,
                            [
                                (MARGIN + CELL_SIZE) * c + MARGIN,
                                (MARGIN + CELL_SIZE) * r + MARGIN,
                                CELL_SIZE,
                                CELL_SIZE,
                            ],
                        )
                    else:
                        pygame.draw.rect(
                            self.screen,
                            color,
                            [
                                (MARGIN + CELL_SIZE) * c + MARGIN,
                                (MARGIN + CELL_SIZE) * r + MARGIN,
                                CELL_SIZE,
                                CELL_SIZE,
                            ],
                        )

            # Draw robot
            rx = (MARGIN + CELL_SIZE) * self.robot_pos[1] + MARGIN + CELL_SIZE // 2
            ry = (MARGIN + CELL_SIZE) * self.robot_pos[0] + MARGIN + CELL_SIZE // 2
            pygame.draw.circle(self.screen, BLUE, (rx, ry), CELL_SIZE // 2 - 5)

            # Draw bottom UI panel
            pygame.draw.rect(
                self.screen, (40, 40, 40), (0, self.height - 100, self.width, 100)
            )

            # Buttons
            c1 = YELLOW if self.mode == "SINGLE_RETURN" else (100, 100, 100)
            c2 = YELLOW if self.mode == "BATCH_TSP" else (100, 100, 100)
            pygame.draw.rect(self.screen, c1, self.btn_single)
            pygame.draw.rect(self.screen, c2, self.btn_batch)
            pygame.draw.rect(
                self.screen, GREEN if self.state == "IDLE" else GRAY, self.btn_run
            )

            # Draw button labels onto button surfaces with blit
            self.screen.blit(
                self.font_ui.render("1. SINGLE RETURN", True, BLACK),
                (self.btn_single.x + 10, self.btn_single.y + 10),
            )
            self.screen.blit(
                self.font_ui.render("2. BATCH (TSP)", True, BLACK),
                (self.btn_batch.x + 25, self.btn_batch.y + 10),
            )
            self.screen.blit(
                self.font_ui.render("RUN MISSION", True, BLACK),
                (self.btn_run.x + 15, self.btn_run.y + 10),
            )

            # Live stats
            stats = f"Energy: {self.total_energy:.1f}  |  Time: {self.total_time:.1f}s"
            self.screen.blit(
                self.font_ui.render(stats, True, WHITE), (20, self.height - 30)
            )

            # When simulation finished, draw summary popup
            if self.state == "FINISHED":
                self.draw_summary_popup()

            pygame.display.flip()
            self.clock.tick(FPS)
        pygame.quit()


if __name__ == "__main__":
    sim = Simulation()
    sim.run()
