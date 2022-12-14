import sys
import yaml

def convert_paths_to_schedule(paths_file, schedule_file):
    paths = open(paths_file).readlines()
    schedule = {}
    for path in paths:
        agent = 'agent' + path.split(':')[0].split(' ')[1]
        schedule[agent] = []
        path = path.split(':')[1].strip()
        path = path.split('->')
        for i in range(len(path)-1):
            ag = {}
            x, y, theta = path[i].strip('()').split(',')
            ag['x'] = int(x)
            ag['y'] = int(y)
            ag['theta'] = int(theta)
            ag['t'] = i
            schedule[agent].append(ag)

    with open(schedule_file, 'w') as outfile:
        yaml.dump({'schedule': schedule}, outfile, default_flow_style=False)

    
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python convert_paths_to_schedule.py paths_file schedule_file')
        exit(1)
    paths_file = sys.argv[1]
    schedule_file = sys.argv[2]
    convert_paths_to_schedule(paths_file, schedule_file)
