### The visualization script needs the map and paths in a yaml file. For this reason, we first convert the maps and the paths to yaml file using the following scripts. 

1. Converting map to yaml file. 
    
python3 convert_map_to_yaml.py    <map_file>    <result_file>    <paths_file>

``` 

python3 convert_map_to_yaml.py ./maps/map2.txt ./results/map_final.yaml ./paths/paths_final.txt

```
2. Converting paths to schedule file. 

python3 convert_paths_to_schedule.py    <paths_file>    <results_file>

```
python3 convert_paths_to_schedule.py ./paths/paths_final.txt ./results/sched_final.yaml 

```
3. To view the visualization

python3 viz.py    <map_yaml_file>    <schedule_yaml_file>
```
python3 viz.py ./results/map_final.yaml ./results/sched_final.yaml         

```
Hit enter after every run of visualization to continue rewatching.
