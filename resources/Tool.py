# sudo chmod 666 /dev/ttyACM0 
class Tool:
    def __init__(self, tool_type="gripper", state="open"):
        if tool_type == "gripper":
            from vac_grip_control.gripper_librairy import open_gripper, close_gripper
            self.tool_close = close_gripper
            self.tool_open = open_gripper
        elif tool_type == "vacuum":
            from vac_grip_control.vacuum_cup_librairy import gripper_strive, gripper_release
            self.tool_close = gripper_strive
            self.tool_open = gripper_release
        else:
            raise ValueError("Invalid tool type")
        self.state = -1
        if state == "open":
            self.open(force=True)
        elif state == "close":
            self.close(force=True)
        else:
            raise ValueError("Invalid tool state")
    
    def open(self, force=False):
        if self.state == 1 or force:
            self.tool_open()
            self.state = 0
    
    def close(self, force=False):
        if self.state == 0 or force:
            self.tool_close()
            self.state = 1
    
    def get_state(self):
        return self.state