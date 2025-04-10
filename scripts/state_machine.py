import rclpy
from rclpy.node import Node

class PlantInspectionFSM(Node):
    def __init__(self):
        super().__init__('plant_inspection_fsm')

        # Initialize flags/state data
        self.state = 'start'
        self.plant_detected = False
        self.arrived_at_plant = False
        self.obstacle_clear = True
        self.photo_quality_good = False

        self.timer = self.create_timer(1.0, self.run_state_machine)

    def run_state_machine(self):
        self.get_logger().info(f'Current State: {self.state}')

        if self.state == 'start':
            self.state = 'localize_plant'

        elif self.state == 'localize_plant':
            if not self.plant_detected:
                self.state = 'scanning_mode'
            else:
                self.state = 'plan_path'
                # Start path planning
                

        elif self.state == 'scanning_mode':
            # Add the scanning mode here
            # add in aruco detection for plant detection
            self.plant_detected = True  
            self.state = 'localize_plant'

        elif self.state == 'plan_path':
            self.state = 'scan_obstacles'
            # have nav2 running for obstacle avoidance and cost mapping

        elif self.state == 'scan_obstacles':
            # have nav2 running for obstacle avoidance and cost mapping
            if self.obstacle_clear:
                self.arrived_at_plant = True
                self.state = 'arrived_at_plant'
            else:
                self.arrived_at_plant = False
                self.state = 'plan_path'  
                # Retry navigation
        elif self.state == 'arrived_at_plant':
            self.state = 'activate_camera'

        elif self.state == 'activate_camera':
            self.state = 'align_plant'
            # work out the fine tuning

        elif self.state == 'align_plant':
            self.state = 'take_photos'
            # photo taking process

        elif self.state == 'take_photos':
            self.state = 'quality_check'

        elif self.state == 'quality_check':
            if self.photo_quality_good:
                self.state = 'store_database'
            else:
                self.state = 'take_photos'

        elif self.state == 'store_database':
            self.state = 'start'  
            # Restart loop and looking for next ID
        else:
            self.get_logger().warn(f'Unknown state: {self.state}')
            self.state = 'start'

def main(args=None):
    rclpy.init(args=args)
    node = PlantInspectionFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
