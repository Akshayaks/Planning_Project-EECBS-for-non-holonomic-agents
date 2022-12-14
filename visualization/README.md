### The visualization script needs the map and paths in a yaml file. For this reason, we first convert the maps and the paths to yaml file using the following scripts. 

1. Converting map to yaml map file:

```   
python3 convert_map_to_yaml.py    <map_file>    <result_file>    <paths_file>

python3 convert_map_to_yaml.py ./maps/map2.txt ./results/map_final.yaml ./paths/paths_final.txt

```
2. Converting paths to yaml schedule file:

```
python3 convert_paths_to_schedule.py    <paths_file>    <results_file>

python3 convert_paths_to_schedule.py ./paths/paths_final.txt ./results/sched_final.yaml 

```
3. To view the visualization on screen:
   - Hit enter after every run of visualization to continue rewatching or 'q' to exit the program and visualization. 

```
python3 viz.py    <map_yaml_file>    <schedule_yaml_file>

python3 viz.py ./results/map_final.yaml ./results/sched_final.yaml         

```

1. To save the visualization to a file: (You will not be able to view on screen in this case)

```
python3 viz.py    <map_yaml_file>    <schedule_yaml_file> --video <video_file.mp4> --speed <num>


python3 viz.py ./results/map_final.yaml ./results/sched_final.yaml --video ./videos/final.mp4  --speed 2
```


