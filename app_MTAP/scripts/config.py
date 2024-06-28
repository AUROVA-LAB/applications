import os
#Config file for variables used by multiple files

PATH_DATASET = os.getenv("PATH_DATASET")
if PATH_DATASET is None: PATH_DATASET = "/home/alolivas/aurova-lab/labrobotica/dataset/CARLA_dataset/Town05_experiment2_session"
ROUTE_FILENAME = os.getenv("CARLA_MAP")
if ROUTE_FILENAME is None: ROUTE_FILENAME = "./routes/Town01.xml" 
PEDESTRIANS_ROUTES_FILE = os.getenv("PEDESTRIAN_ROUTES")
if PEDESTRIANS_ROUTES_FILE is None: PEDESTRIANS_ROUTES_FILE="routes/pedestrian_routes/Town01_evaluation5.xml"
WAY_ID = os.getenv("CARLA_WAY")
if WAY_ID is None: WAY_ID = 14
else: WAY_ID = int(WAY_ID)
PATH_RESULTS = os.getenv("CARLA_RESULTS_PATH")
if PATH_RESULTS is None: PATH_RESULTS ="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/results_metrics/"

PENALTY_COLLISION_SLIGHT=0.9
PENALTY_COLLISION_STATIC=0.65
PENALTY_COLLISION_PEDESTRIAN=0.5
PENALTY_OFF_ROAD=0.7 #In this case the penalty is on "cars" road driving, except pedestrian crossing. Therefore, we penalize every crossed road line.
TIME_DISTANCE_COEFFICIENT = 3 #Coefficient for determining the max trajectory time
MAX_TIME_LOCAL_MINIMUM = 30 #Seconds
MINIMUM_DISTANCE = 3.0
DISTANCE2PEDESTRIAN = 1.0 #Distance to the pedestrian in the direction of movement to consider that it has been avoided.


#Ackermann control configuration
MAX_SPEED=1.3 # m/s
MAX_STEERING_ANGLE = 24 # degrees
KP = 2.0
KI = 0.5
KD = 0.5

DEG2RAD = 3.1415927/180.0
FPS=10