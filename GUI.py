import streamlit as st
import subprocess
import re
import shlex
import time

def get_available_agents():
    # Run the ros2 service list command
    result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True)
    
    # Parse the output to find services with 'agent_' prefix and 'update_goal' suffix
    services = result.stdout.split('\n')
    agent_services = [s for s in services if s.startswith('/agent_') and s.endswith('/update_goal')]
    
    # Extract agent names
    agents = [re.search(r'/agent_(\w+)/', s).group(1) for s in agent_services]
    
    return agents


def stresstesting(available_agents,testRuns=1):
    available_agents = 3
    start_point_list_1 = [[0, 0, 0], [2, 0, 0], [4, 0, 0]]
    destination_point_list_1 = [[9, 9, 0], [9, 9, 1], [9, 9, 2]]
    start_point_list_2 = [[9, 9, 0], [9, 9, 1], [9, 9, 2]]
    destination_point_list_2 = [[1, 9, 1], [9, 1, 1], [1, 1, 1]]
    start_point_list_3 = [[1, 9, 1], [9, 1, 1], [1, 1, 1]]
    destination_point_list_3 = [[0, 0, 0], [2, 0, 0], [4, 0, 0]]
    for i in range(testRuns):
        for j in range(1,available_agents+1): # goes from 1 to 3
            call_update_goal_service(str(j), destination_point_list_1[j-1][0], destination_point_list_1[j-1][1], destination_point_list_1[j-1][2])
            #wait for 5 seconds
            time.sleep(4)

        for j in range(1,available_agents+1): # goes from 1 to 3
            call_update_goal_service(str(j), destination_point_list_2[j-1][0], destination_point_list_2[j-1][1], destination_point_list_2[j-1][2])
            #wait for 5 seconds
            time.sleep(4)

        for j in range(1,available_agents+1): # goes from 1 to 3
            call_update_goal_service(str(j), destination_point_list_3[j-1][0], destination_point_list_3[j-1][1], destination_point_list_3[j-1][2])
            #wait for 5 seconds
            time.sleep(4)


def call_update_goal_service(agent, x, y, z):
    yaml_args = f"{{goal_pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}"
    
    command = [
        'ros2', 'service', 'call',
        f'/agent_{agent}/update_goal',
        'my_robot_interfaces/srv/UpdateGoal',
        shlex.quote(yaml_args)
    ]
    
    # Join the command parts into a single string
    command_str = ' '.join(command)
    
    # Use shell=True to properly handle the YAML string
    result = subprocess.run(command_str, shell=True, capture_output=True, text=True)
    return result.stdout, result.stderr

st.title('Agent Goal Update GUI')


# Get available agents
available_agents = get_available_agents()

# Agent selection
selected_agent = st.selectbox('Select an agent:', available_agents)

# Target destination input
st.subheader('Set Target Destination')
x = st.number_input('X coordinate:', value=0.0)
y = st.number_input('Y coordinate:', value=0.0)
z = st.number_input('Z coordinate:', value=0.0)

# Execute button
# Execute button
if st.button('Update Goal'):
    stdout, stderr = call_update_goal_service(selected_agent, x, y, z)
    if stderr:
        st.error(f'Error occurred: {stderr}')
    else:
        st.success(f'Goal updated for agent_{selected_agent}')
        st.text(f'Service response:\n{stdout}')

num_tests = st.number_input('Number of tests:', value=1)
if st.button("stress Test"):
    stresstesting(available_agents,num_tests)
    st.success("stress test completed")