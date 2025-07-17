import roplat_exrobot as ex

robot = ex.ExRobot()

# robot before initialization 

robot.init()

# robot after initialization can execute non-motorized operation instructions

robot.enable()

# robot after enabling can execute motorized operation instructions

robot.disable()

# robot after disabling can execute non-motorized operation instructions

robot.shutdown()

# robot after shutdown will not respond to any instructions except for initialization