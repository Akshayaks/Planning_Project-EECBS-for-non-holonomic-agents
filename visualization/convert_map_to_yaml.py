import sys
import yaml

def convert_map_to_yaml(map_file, yaml_file, path_file):
    # read map file
    map_data = open(map_file).readlines()
    # get map size
    map_height = map_data[1].split()
    map_width = map_data[2].split()
    height = int(map_height[1])
    width = int(map_width[1])
    # get map data
    map_data = map_data[4:]
    map_data = [line.strip() for line in map_data]
    # get obstacles
    obstacles = []
    for i in range(height):
        for j in range(width):
            if map_data[i][j] == '@':
                obstacles.append([i, j])
    
    # extract start and goal from paths.txt file
    paths = open(path_file).readlines()
    starts = []
    goals = []
    for path in paths:
        path = path.split('->')
        start = path[0].split(')')[0].split('(')[1].split(',')
        goal = path[-2].split(')')[0].split('(')[1].split(',')
        starts.append([int(start[0]), int(start[1])])
        goals.append([int(goal[0]), int(goal[1])])

    # write yaml file
    yaml_data = {}
    yaml_data['map'] = {}
    yaml_data['map']['dimensions'] = [height, width]
    yaml_data['map']['obstacles'] = obstacles 
    yaml_data['agents'] = []
    for i in range(len(starts)):
        agent = {} # number and name agents from 0
        agent['start'] = starts[i]
        agent['goal'] = goals[i]
        agent['name'] = 'agent'+str(i) 
        yaml_data['agents'].append(agent)
    with open(yaml_file, 'w') as outfile:
        yaml.dump(yaml_data, outfile, default_flow_style=None)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: python convert_map_to_yaml.py <map_file> <yaml_file> <path_file>')
        sys.exit(1)
    convert_map_to_yaml(sys.argv[1], sys.argv[2], sys.argv[3])
