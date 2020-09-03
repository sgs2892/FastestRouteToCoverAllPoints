from PIL import Image
import sys
from queue import PriorityQueue

total_Distance = 0

class Node():
    __slots__ = ["parent", "positionX", "positionY", "time_to_reach"]

    def __init__(self, positionX, positionY, parent=None, time_to_reach=0):
        self.parent = parent
        self.positionX = positionX
        self.positionY = positionY
        self.time_to_reach = time_to_reach

    def __eq__(self, other):
        return self.positionX == other.positionX and self.positionY == other.positionY


def read_elevation_file(elevation_file):
    with open(elevation_file) as elevations:
        return [[float(height) for height in row.split()[:-5]] for row in elevations]


def getSpeed(terrain, season):
    if terrain == (5, 73, 24, 255) or terrain == (0, 0, 255, 255) or terrain == (205, 0, 101, 255) or terrain == (107, 62, 0, 255):
        return 0
    elif terrain == (2, 136, 40, 255):
        return 1.0
    elif terrain == (255, 192, 0, 255) or terrain == (2, 208, 60, 255):
        return 2.0
    elif terrain == (255, 255, 255, 255):
        if season == "fall":
            return 0.5
        return 2.5
    elif terrain == (0, 0, 0, 255) or terrain == (248, 148, 18, 255):
        return 3.0
    elif terrain == (71, 51, 3, 255):
        return 4.0
    elif terrain == (153, 255, 255, 255):
        return 0.5


def calaulate_Heuristic(next_node, goal_point):
    return ((next_node.positionX - goal_point.positionX) ** 2) + ((next_node.positionY - goal_point.positionY) ** 2)


def calculate_cost(terrain_file, elevation_file, current_point, next_node, moving_towards, goal_point, travel_time, season):
    current_Terrain = terrain_file[current_point[0], current_point[1]]
    current_Elevation = elevation_file[current_point[1]][current_point[0]]
    next_Terrain = terrain_file[next_node.positionX, next_node.positionY]
    next_Elevation = elevation_file[next_node.positionY][next_node.positionX]
    altitude = next_Elevation - current_Elevation

    if moving_towards == 'h':
        distance = 7.55
    elif moving_towards == 'v':
        distance = 10.29
    else:
        distance = 12.76 #math.sqrt(7.55**2 + 10.29**2)
    steep = (altitude/distance)*100
    current_speed = getSpeed(current_Terrain, season)
    next_speed = getSpeed(next_Terrain, season)
    if current_speed == 0 or next_speed == 0:
        time = float('inf')
    else:
        time = ((distance/2) / current_speed) + ((distance/2) / next_speed)
    if steep > 0:
        time += (distance * steep)
    elif steep < 0:
        time -= (distance * steep)
    next_node.time_to_reach = travel_time[(current_point[0], current_point[1])] + time
    travel_time[(next_node.positionX, next_node.positionY)] = next_node.time_to_reach
    global total_Distance
    total_Distance += distance
    return next_node.time_to_reach + calaulate_Heuristic(next_node, goal_point)


def A_star_path(terrain_file, elevation_file, start_point, goal_point, season):
    traversal_list = PriorityQueue()
    traversal_list.put((0, (start_point.positionX, start_point.positionY)))
    parent_set = {(start_point.positionX, start_point.positionY): None}
    travel_time = {}
    travel_time[(start_point.positionX, start_point.positionY)] = 0
    while not traversal_list.empty():
        current_point = traversal_list.get()[1]
        if current_point[0] == goal_point.positionX and current_point[1] == goal_point.positionY:
            return travel_time[(goal_point.positionX, goal_point.positionY)], parent_set
        for direction in [[0, -1, 'v'], [0, 1, 'v'], [-1, 0, 'h'], [1, 0, 'h'], [1, 1, 'd'], [-1, 1, 'd'], [-1, -1, 'd'], [1, -1, 'd']]:
            possible_point = [current_point[0] + direction[0], current_point[1] + direction[1]]
            if possible_point[0] < 0 or possible_point[0] > 394 or possible_point[1] < 0 or possible_point[1] > 499:
                continue
            if (possible_point[0], possible_point[1]) not in parent_set:
                next_node = Node(int(possible_point[0]), int(possible_point[1]))
                moving_towards = direction[2]
                traversal_list.put((calculate_cost(terrain_file, elevation_file, current_point, next_node, moving_towards, goal_point, travel_time, season), (next_node.positionX, next_node.positionY)))
                next_node.parent = current_point
                parent_set[(next_node.positionX, next_node.positionY)] = current_point


def draw_Path(output_Map, parent_set, points, outPut_File):
    output_Map_plot = output_Map.load()
    while True:
        output_Map_plot[points[0], points[1]] = (255, 0, 0, 255)
        if parent_set[(points[0], points[1])] is None:
            break
        points = parent_set[(points[0], points[1])]
    output_Map.save(outPut_File)


def perform_bfs(terrain_file_pixel, position, depth, season, safe_zone, elevation_file, water_elevation, max_depth):
    traverse_list = []
    traverse_list.append((position, depth))
    while traverse_list:
        location = traverse_list.pop(0)
        point = location[0]
        current_depth = location[1]
        if current_depth > max_depth:
            continue
        for direction in [[0, -1], [0, 1], [-1, 0], [1, 0], [1, 1], [-1, 1], [-1, -1], [1, -1]]:
            if point[1] + direction[0] >= 0 and point[1] + direction[0] <= 394 and point[0] + direction[1] >= 0 and point[0] + direction[1] <= 499:
                if season == "winter":
                    if terrain_file_pixel[point[1] + direction[0], point[0] + direction[1]] == (0, 0, 255, 255):
                        terrain_file_pixel[point[1] + direction[0], point[0] + direction[1]] = (153, 255, 255, 255)
                        traverse_list.append(((point[0] + direction[1], point[1] + direction[0]), current_depth + 1))
                elif season == "spring":
                    if terrain_file_pixel[point[1] + direction[0], point[0] + direction[1]] in safe_zone:
                        if elevation_file[point[0] + direction[1]][point[1] + direction[0]] - water_elevation <= 5.0:
                            terrain_file_pixel[point[1] + direction[0], point[0] + direction[1]] = (107, 62, 0, 255)
                            traverse_list.append(((point[0] + direction[1], point[1] + direction[0]), current_depth + 1))
    return terrain_file_pixel


def make_changes(terrain_file_pixel, column_pixel, row_pixel, safe_zone, season, elevation_file, water_elevation):
    for direction in [[0, -1], [0, 1], [-1, 0], [1, 0], [1, 1], [-1, 1], [-1, -1], [1, -1]]:
        if column_pixel + direction[0] < 0 or column_pixel + direction[0] > 394 or row_pixel + direction[1] < 0 or row_pixel + direction[1] > 499:
            continue
        if terrain_file_pixel[column_pixel+direction[0], row_pixel+direction[1]] in safe_zone:
            terrain_file_pixel = perform_bfs(terrain_file_pixel, (row_pixel, column_pixel), 0, season, safe_zone, elevation_file, water_elevation, max_depth=15 if season == "spring" else 3)
    return terrain_file_pixel


def change_Map(terrain_file, season, elevation_file):
    terrain_file_pixel = terrain_file.load()
    safe_zone = [(2, 136, 40, 255), (255, 192, 0, 255), (2, 208, 60, 255), (255, 255, 255, 255), (0, 0, 0, 255),
                 (248, 148, 18, 255), (71, 51, 3, 255)]
    for row_pixel in range(500):
        for column_pixel in range(395):
            if terrain_file_pixel[column_pixel, row_pixel] == (0, 0, 255, 255):
                terrain_file_pixel = make_changes(terrain_file_pixel, column_pixel, row_pixel, safe_zone, season, elevation_file, elevation_file[row_pixel][column_pixel])


def main():
    terrain_Image = sys.argv[1]
    elevation_Text = sys.argv[2]
    path_file = sys.argv[3]
    season = sys.argv[4]
    outPut_File = sys.argv[5]

    terrain_file = Image.open(terrain_Image).load()
    elevation_file = read_elevation_file(elevation_Text)
    output_Map = Image.open(terrain_Image)
    if season == "winter" or season == "Winter" or season == "Spring" or season == "spring":
        change_Map(output_Map, season.lower(), elevation_file)
        output_Map.save(outPut_File)
        terrain_file = Image.open(terrain_Image).load()
        output_Map = Image.open(outPut_File)
    with open(path_file) as travel_points:
        first_point = travel_points.readline().split()
        source = Node(int(first_point[0]), int(first_point[1]))
        checkpoints = []
        for positions in travel_points:
            co_ordinates = positions.split()
            checkpoints.append(Node(int(co_ordinates[0]), int(co_ordinates[1])))
        start_point = source
        time_required = 0
        for points in checkpoints:
            time_to_reach, parent_set = A_star_path(terrain_file, elevation_file, start_point, points, season)
            draw_Path(output_Map, parent_set, (points.positionX, points.positionY), outPut_File)
            time_required += time_to_reach
            start_point = points
    print("Total time required: %d seconds" % time_required)
    print("Total distance covered: %.2f meters" % total_Distance)


if __name__ == "__main__":
    main()