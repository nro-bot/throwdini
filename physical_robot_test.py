from pyUR import PyUR

r = PyUR(tcp_host_ip, self.joint_vel,
              self.joint_acc, home_joint_config=home_joint_config,
              workspace_limits=workspace_limits)

# Move robot to home pose

# activate_gripper never used

is_sim: never sends the data
however: can we ask it to not have to connect to the robot at all? to simulate
training the
camera independently for instance 


Also -- we have to decide for the move to be guarded or not
And -- where should the joint accelerations and velocities be declared?

    def move_to(self, position, orientation, vel=None, acc=None, radius=0,
                wait=True, override_safety=False):

There are two catches: is it safe to move somewhere?
This is complicated by the fact that we want margins on the z axis! 


we should only have one move_to or move_joints function (combine them) -- only
the former has is_safe because I never figured out how to translate position
into joints (I guess need my own IK)


