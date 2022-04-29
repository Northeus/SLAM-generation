# SLAM-generation
Generate simple data for SLAM estimators.

-------------------------------------------------------------------------------

## Data sources
We used groundtruth data from
[snapdragon dataset #3](https://fpv.ifi.uzh.ch/datasets/) (`groundtruth.txt`),
which have to be placed in `data` folder.

-------------------------------------------------------------------------------

## Data files
 - files stored in `data` folder used for generation

### Groundtruth
 - `groundtruth.txt` contains groundtruth data from snapdragon dataset.
 - file have one header line
 - data are decimal numbers separated with space:
   * timestamp in seconds
   * three numbers representing coordinates
   * four numbers representing quaternion (x, y, z, w)

### Walls
 - `walls.csv`
 - contains information used for generating point clouds
 - data are decimal numbers separated with space:
   * three numbers representing coordinates of the corner of the wall
   * three numbers representing coordinates of the first adjenct corner
   * three numbers representing coordinates of the second adjenct corner
   * number of points in the given wall (numbers' decimal part will be removed)

### Output positions
 - `output_positions.csv` contains the ground truth positions
 - data are three decimal numbers representing coordinates separated with space

### Output points
 - `output_points.csv` contains the positions of the points
 - data are three decimal numbers representing coordinates separated with space

### Output seen
 - `output_seen.csv` contains visible positions
 - data are two integers separated with space:
  * first integer represents the line number of the position used for testing,
    whether the camera sees the point (from `output_positions.csv`)
  * second integer represents the line number of the point that was seen by
    the camera (from `output_points.csv`)

-------------------------------------------------------------------------------

## Scripts
 - `. scripts/run.sh` generate data

