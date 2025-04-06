import pygame
import threading
from nav_controller import NavManager
from util import RobotStatePublisher
from state_pubsub.state_sub import RobotStateSub, BotEntry, RobotState
import rclpy
from dataclasses import dataclass
import time
import csv


class RobotManager():
    """
    Starts robot_state subscriber
    Stores a list of robots, as well as their current state.\n
    Checks bots for nav instances, and creates them in separate thread if they do not exist
    Loads robot details and product details from config
    Navigate and Search functions which run on a nav manager in a new thread


    Class Variables:
    - `bots` (list[BotEntry]): Tracks available robots and holds relevant information about them (state)
    """
    def __init__(self):
        

        self.sub = RobotStateSub()
        self.pub = RobotStatePublisher()

        # Robot ping timeout - this is seconds/60, assuming a pygame framerate of 60
        self.ping_timeout = 15 * 60 # check every 15 seconds - online status should be published every 5 seconds for each robot, so ~3 chances to not be set to offline

        #start subscriber in different thread
        ros_thread = threading.Thread(target=self.start_subscriber, args=(self.sub,), daemon=True)
        ros_thread.start()

        #Create nav instances if they do not exist (Now implemented with PyGame timer)
        # self.timer_thread = threading.Thread(target=self.update_bots, daemon=True)
        # self.timer_thread.start()

        # Load robot information into robot_details dictionary
        self.robots_config = [dict]
        self.load_config()

        # Load product information
        self.products = [dict]
        self.load_products()

        

    def set_robots_pinging(self):
        for b in self.sub.bots:
            if b.state == RobotState.ONLINE:
                print(f"Set {b.name} to pinging")
                b.state = RobotState.PINGING


    
    def check_bots_offline(self):
        for b in self.sub.bots:
            if b.state == RobotState.PINGING:
                print(f"Bot {b.name} set to offline!")
                b.state = RobotState.OFFLINE


    def reset_battery_state_timer(self):
        self.sub.battery_state_set.update({key : False for key in self.sub.battery_state_set})

        print(f"Current battery states: {dict}")
        print("Reset battery state timers!")



    def load_config(self):
        with open("warehouse_robots_config.csv", newline="", encoding="utf-8") as csvfile:

            # Add each robot in csv file and add to robot_entry
            reader = list(csv.reader(csvfile))
            for row in reader[1:]:
                self.robots_config.append(
                    {
                        "name" : row[0],
                        "start_pos" : (row[1],row[2],row[3]),
                        "delivery_pos" : (row[4],row[5],row[6])
                    }
                )

    def load_products(self):
        with open("warehouse_products.csv", newline="", encoding="utf-8") as csvfile:

            # Add each robot in csv file and add to robot_entry
            reader = list(csv.reader(csvfile))
            for row in reader[1:]:
                self.products.append(
                    {
                        "product_id" : row[0],
                        "pos" : (row[1],row[2],row[3])
                    }
                )


    def search_config(self, search_name : str) -> dict:

        for c in self.robots_config:
            if c["name"] == search_name:
                return c
            
        print("Search term not found!")
        return {}
            
    
    def search_products(self, name : str) -> dict:

        for c in self.products:
            if c["product_id"] == name:
                return c
            
        print("Search term not found!")
        return {}

    def start_subscriber(self, node):
        rclpy.spin(node)

    def stop_subscriber(self):
        self.sub.destroy_node()
        rclpy.shutdown()

    def print_bots(self):
        print(self.sub.bots)

    def update_bots(self):
    
        for bot in self.sub.bots:
            if bot.nav_manager == None:
                bot_details = self.search_config(bot.name)

                if not bot_details:
                    print(self.robots_config)
                    raise Exception(f"Bot {bot.name} not found in config file!")
                
                bot.nav_manager = NavManager(bot_details["start_pos"], bot_details["delivery_pos"], f"{bot.name}", self.pub)
                
            

    def navigate_bot(self, bot : BotEntry, pos : tuple[float, float, float]):
        nav_thread = threading.Thread(target=bot.nav_manager.navigate_to_position, args = (pos[0],pos[1],0))
        nav_thread.daemon = True
        nav_thread.start()

    def fetch_product(self, bot : BotEntry, pos : tuple[float, float, float]):
        nav_thread = threading.Thread(target=bot.nav_manager.fetch_item, args = (pos[0],pos[1],0))
        nav_thread.daemon = True
        nav_thread.start()

    def search_bot(self, name : str) -> BotEntry:
        bot : BotEntry
        for b in self.sub.bots:
            if b.name == name:
                return b
        
        print(f"Bot {name} not found!")
        return None
        
#region pygame elements


# constants
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
DARK_GRAY = (150, 150, 150)
RED = (255,0,0)
GREEN = (0,255,0)
ALGAE = (96, 108, 56)
DARK_GREEN = (40, 54, 24)
CREAM = (254, 250, 224) 
#font = pygame.font.Font(None, 40)


class Button():
    def __init__(self, x, y, width, height, text, base_color, hover_color):
        
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.font = font = pygame.font.Font(None, 40)
        self.base_color = base_color
        self.hover_color = hover_color

    def draw(self, screen):
        # Check if the mouse is hovering over the button
        mouse_pos = pygame.mouse.get_pos()
        if self.rect.collidepoint(mouse_pos):
            pygame.draw.rect(screen, self.hover_color, self.rect)
        else:
            pygame.draw.rect(screen, self.base_color, self.rect)

        # Draw the text
        text_surface = self.font.render(self.text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def is_clicked(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True
        return False




class StatusBar:
    def __init__(self, x, y, width = 500, height = 100, name="Robot", battery=0, bot = BotEntry):
        self.x, self.y = x,y
        self.rect = (x, y, width, height)
        self.name = name
        self.font = pygame.font.Font(None, 40)
        self.battery = battery 
        self.bot = bot
        self.nav_states = [RobotState.NAVIGATING, RobotState.NAV_DONE, RobotState.NAV_ERROR]
        self.last_state = ""
        self.last_nav_state = ""

      
    def draw(self, surface):
        # Draw status bar background
        pygame.draw.rect(surface, CREAM, self.rect, border_radius=10)

        # Draw name
        name_text = self.font.render(f"Name: {self.name}", True, BLACK)
        surface.blit(name_text, (self.x + 10, self.y + 10))

          # Draw nav_status text
        if self.bot.state in self.nav_states:
            self.last_nav_state = RobotState(self.bot.state).name
        else:
            self.last_state = RobotState(self.bot.state).name

       
        status_text = self.font.render(f"Status: {self.last_state}", True, BLACK)

        nav_status_text = self.font.render(f"Nav: {self.last_nav_state}", True, BLACK)

        surface.blit(status_text, (self.x + 10, self.y + 40))
        surface.blit(nav_status_text, (self.x + 250, self.y + 40))

        # Draw battery bar outline
        battery_x = self.x + 10
        battery_y = self.y + 70
        battery_width = 200
        battery_height = 25
        pygame.draw.rect(surface, BLACK, (battery_x, battery_y, battery_width, battery_height), 2)

        # Fill battery level
        battery_fill_width = int((self.battery / 100) * (battery_width - 4))
        battery_fill_color = GREEN if self.battery > 20 else RED
        pygame.draw.rect(surface, battery_fill_color, (battery_x + 2, battery_y + 2, battery_fill_width, battery_height - 4))

        # Display battery percentage
        self.update_battery(self.bot.battery_state)
        battery_text = self.font.render(f"Battery: {self.battery}%", True, BLACK)
        surface.blit(battery_text, (battery_x + battery_width + 10, battery_y))


    def update_battery(self, value : float):
        self.battery = max(1, min(100, int(value)))


    
class Console:
    def __init__(self, x, y, width, height, bot_manager : RobotManager):
        self.x, self.y, self.width, self.height = x, y, width, height
        self.font = pygame.font.Font(None, 40)
        self.text = ""
        self.output_text = ""
        self.active = False
        self.bot_manager = bot_manager
        self.output_text_colour = RED

    def draw(self, surface):
        pygame.draw.rect(surface, BLACK, (self.x, self.y, self.width, self.height))
        pygame.draw.rect(surface, WHITE, (self.x + 2, self.y + 2, self.width - 4, self.height - 4))
        
        # Draw text
        text_surface = self.font.render(self.text, True, BLACK)
        error_text_surface = self.font.render(self.output_text, True, self.output_text_colour)
        surface.blit(text_surface, (self.x + 5, self.y + 5))
        surface.blit(error_text_surface, (self.x + 5, self.y + 50))


    def handle_event(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self.process_command(self.text)
                self.text = "" #clear input

            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]

            # Add key (which the user pressed)
            else:
                self.text += event.unicode

    def process_command(self, command):
        if command.lower().startswith("navigate") or command.lower().startswith("fetch"):
            try:
                split = command.lower().split(" ")
                
                if len(split) < 3:
                    self.output_text = f"{split[0]} command takes 2 arguments! use the format {split[0]} <bot_name> <(x,y,z)>"
                    self.output_text_colour = RED
                    return
                
                bot = self.bot_manager.search_bot(split[1])

                if not bot:
                    self.output_text = f"Robot {split[1]} could not be found!"
                    self.output_text_colour = RED
                    return

                
                if split[0].lower() == "navigate":

                    pos = split[2]
                    pos_tuple = tuple(float(x) for x in pos[1:-1].split(','))

                    self.bot_manager.navigate_bot(bot, pos_tuple)
                    self.output_text = f"Navigating robot {split[1]} to position {split[2]}"
                    self.output_text_colour = GREEN

                elif split[0].lower() == "fetch":
                    product_id = split[2]
                    product = self.bot_manager.search_products(product_id)
                    if not product:
                        self.output_text = f"Product {split[2]} not found in products file!"
                        self.output_text_colour = RED

                    
                    self.bot_manager.fetch_product(bot, product["pos"])
                    self.output_text = f"Robot {split[1]} fetching item {product_id}!"
                    self.output_text_colour = GREEN

            except:
                raise Exception("command has encountered an error")
            
        else:
            self.output_text = f"Unrecognized command..."
            self.output_text_colour = RED


 
#endregion


# Main loop
def main():

    pygame.init()


    # Screen dimensions
    SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 1000
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Warehouse")


    clock = pygame.time.Clock()
    running = True
    mainpage = True
    button_list = []
    fps_timeout_timer = 0
    rclpy.init()

    bot_manager = RobotManager()

    #region navbarinit
    nav_pad = 20
    nav_x, nav_y, nav_width, nav_height = 0, 0, SCREEN_WIDTH, 120
    button_nav_main = Button(nav_pad, nav_pad, 150, 80, "Home", CREAM, DARK_GRAY)

    button_list.append(button_nav_main)


    #button_refresh = Button(200, SCREEN_HEIGHT - 200, 150, 80, "RESET", font, GREEN, DARK_GRAY)
    button_stop = Button(50, SCREEN_HEIGHT - 200, 150, 80, "STOP", RED, DARK_GRAY)


    # button list
    button_list.append(button_stop)
    
    #endregion

    #region robot status

    # Declare positions for consecutive status bars to go
    y_lim : int = SCREEN_HEIGHT - 200
    x_lim : int = SCREEN_WIDTH
    status_pos : tuple[int, int] = []
    status_list : StatusBar = []
    status_gap = 30
    status_width = 500
    status_height = 100
    y_app = status_height + status_gap
    x_app = status_width + status_gap

    s_x = 80
    s_y = 150
    status_pos.append((s_x, s_y))
    s_y += y_app
    # Populate status positions
    while not s_y + status_gap + status_height > y_lim and not s_x + status_gap + status_width > x_lim:
        # add position to list, and increment row
        status_pos.append((s_x, s_y))
        s_y += y_app

        # If we have reached the bottom of current column, increment column and reset y pos
        if s_y + status_gap + status_height > y_lim:
            s_x += x_app
            s_y = 150

    
            
    #console
    console = Console(0, SCREEN_HEIGHT-100, SCREEN_WIDTH, 100, bot_manager)
    #endregion
    
    
    while running:
        screen.fill(ALGAE)

        #events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            #region mainpage
            if button_stop.is_clicked(event):
                print("STOPPING")
                pygame.quit()

            if event.type == pygame.KEYDOWN:
                console.handle_event(event)

            #endregion

        
        if fps_timeout_timer % bot_manager.ping_timeout == 0:
            fps_timeout_timer = 0
            # Add navigation manager to bots if it does not already exist
            bot_manager.update_bots()

            # Set robots to pinging, and check for existing pinging robots
            bot_manager.check_bots_offline()
            bot_manager.set_robots_pinging()

            # Reset battery state sent timer, allowing new battery states to be sent
            bot_manager.reset_battery_state_timer()

            # Check for new robots, and add a status tab if found
            bots = bot_manager.sub.bots
            status_bot_names = [s.name for s in status_list]
            for i, b in enumerate(bots):
                if len(status_list) >= len(status_pos):
                    print("Not enough positions available!")
                else:
                    if not b.name in status_bot_names:
                        status_list.append(StatusBar(status_pos[i][0], status_pos[i][1], name = b.name, bot=b))

        #Nav bar
        pygame.draw.rect(screen, DARK_GREEN, (nav_x, nav_y, nav_width, nav_height))

        
        if mainpage:
            # drawing
            #screen.blit(pgm_surface, (400, 150))

            for button in button_list:
                button.draw(screen)

            #status
            #pygame.draw.rect(screen, CREAM, (status_x, status_y, status_width, status_height))
            for s in status_list:
                s.draw(screen)

            #console
            console.draw(screen)
            
        pygame.display.flip()

        # Increment timer
        fps_timeout_timer += 1
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
