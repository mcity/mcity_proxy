from __future__ import print_function
import time

# from __future__ import print_function

import roslibpy
import roslibpy.actionlib

client = roslibpy.Ros(host="127.0.0.1", port=9090)
client.run()

# service = roslibpy.Service(client, '/set_chassis_enable', 'segway_msgs/srv/RosSetChassisEnableCmd')
# request = roslibpy.ServiceRequest(values={'ros_set_chassis_enable_cmd': True})

# print('Calling service...')
# result = service.call(request)
# print(f'Service response: {result}')

service = roslibpy.Service(
    client,
    "/action_manager/move_distance_send_goal",
    "mcity_proxy_msgs/srv/MoveDistanceSendGoal",
)
request = roslibpy.ServiceRequest(
    values={"move_distance_goal": {"meters_per_second": -0.5, "meters": 1}}
)

print("Calling service...")
result = service.call(request)
print(f"Service response: {result}")

service = roslibpy.Service(
    client,
    "/action_manager/move_distance_cancel_goal",
    "mcity_proxy_msgs/srv/MoveDistanceCancelGoal",
)
request = roslibpy.ServiceRequest(values={})

print("Calling service...")
result = service.call(request)
print(f"Service response: {result}")

# action_client = roslibpy.actionlib.ActionClient(
#     client, "/move_distance", "mcity_proxy_msgs/action/MoveDistance"
# )
# goal = roslibpy.actionlib.Goal(
#     action_client, roslibpy.Message({"meters_per_second": 0.25, "meters": 1})
# )

# goal.on("feedback", lambda f: print(f))
# action_client.add_goal(goal)
# goal.send()
# result = goal.wait(timeout=10)
# if goal.is_finished:
#     print(f'Action finished: {result}')
# else:
#     goal.cancel()
# action_client.dispose()

client.terminate()
