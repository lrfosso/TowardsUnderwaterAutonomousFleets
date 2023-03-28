I mpc_controller noden        
        
        self.declare_parameter('main_id')
        self.declare_parameter('fleet_quantity')
        self.main_id = self.get_parameter('main_id').get_parameter_value().string_value
        self.fleet_quantity = self.get_parameter('fleet_quantity').get_parameter_value().string_value
        
        for id in range(int(self.fleet_quantity)):
            if int(self.main_id) != id:
                self.fleet_id = str(id+2)