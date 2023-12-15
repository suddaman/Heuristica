import sys
import heapq
import time
import pandas as pd

# bfs helper function
def bfs(start_x, start_y, end_x, end_y, mp):
    queue = [(start_x, start_y, 0, [(start_x, start_y)])]
    visited = [[False for _ in range(len(mp[0]))] for _ in range(len(mp))]
    visited[start_x][start_y] = True
    while queue:
        x, y, cnt, path = queue.pop(0)
        if x == end_x and y == end_y:
            return cnt, path
        for i, j in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
            if 0 <= i < len(mp) and 0 <= j < len(mp[0]) and not visited[i][j] and mp[i][j] != 'X':
                if mp[i][j] in ('P', 'N', 'C', 'CC', 'CN'):
                    queue.append((i, j, cnt+1, path + [(i, j)]))
                else:
                    queue.append((i, j, cnt+int(mp[i][j]), path + [(i, j)]))
                visited[i][j] = True
    return -1

class Node():
    def __init__(self, x, y, type):
        self.x = x
        self.y = y
        self.type = type

    # generate adjacent list
    def generate_adjacent_list(self, mp):
        adjacent_list = []

        # for each node, generate adjacent list
        for i in range(len(mp)):
            for j in range(len(mp[i])):
                if self.type != 'C':
                    if mp[i][j] in ('P', 'N', 'C', 'CC', 'CN') and not(i == self.x and j == self.y):
                        adjacent_list.append((i, j, bfs(self.x, self.y, i, j, mp)[0], mp[i][j]))
                if self.type == 'C':
                    if mp[i][j] in ('P', 'CC', 'C') and not(i == self.x and j == self.y):
                        adjacent_list.append((i, j, bfs(self.x, self.y, i, j, mp)[0], mp[i][j]))
        
        self.adjacent_list = adjacent_list


    def __str__(self):
        return 'Node: ({}, {}, {})'.format(self.x, self.y, self.type)

# CarState class
class CarState():

    def __init__(self, energy, C_num, N_num, current_node, pre_car, transported_num):
        self.energy = energy
        self.C_num = C_num
        self.N_num = N_num
        self.current_node = current_node
        self.pre_car = pre_car
        self.transported_num = transported_num

    def __str__(self):
        return 'CarState: ({}, {}, {}, {}, {}, {})'.format(self.energy, self.C_num, self.N_num, self.current_node, self.pre_car, self.transported_num)

    # define the less than function
    def __lt__(self, other):
        return self.energy < other.energy

class AStar():
    def __init__(self, mp, heuristic_function_id):
        self.mp = mp
        self.heuristic_function_id = heuristic_function_id

    def map_to_node(self):
        node_list = []
        for i in range(len(self.mp)):
            for j in range(len(self.mp[i])):
                if self.mp[i][j] in ('P', 'N', 'C', 'CC', 'CN'):
                    node_list.append(Node(i, j, self.mp[i][j]))
        for node in node_list:
            node.generate_adjacent_list(self.mp)
            # print(node)
            # print(node.adjacent_list)
            # print('---------------------------------')
        self.node_list = node_list
    
    def a_star_for_tsp(self):
        def heuristic_function_1(car_state):
            return car_state.energy / 50
        def heuristic_function_2(car_state):
            return 1 / (car_state.transported_num + 1)

        start_node = None
        for node in self.node_list:
            if node.type == 'P':
                start_node = node
                break
        
        C_N_num = 0
        for node in self.node_list:
            if node.type == 'C' or node.type == 'N':
                C_N_num += 1

        # print("C_N_NUM:", C_N_num)

        open_set = []

        heapq.heappush(open_set, (0, CarState(50, 0, 0, start_node, None, 0), []))

        loop_cnt = 0
        while True:
            loop_cnt += 1
            loop_flag = False
            if len(open_set) == 0:
                print("fail")
                loop_flag = True
            if loop_flag:
                return None, loop_cnt
                break

            priority, car_state, visited_c_n = heapq.heappop(open_set)

            #  If the current vehicle does not have enough energy to return to point P, the next step is not taken
            no_energy = False
            for next_node in car_state.current_node.adjacent_list:
                if next_node[3] == 'P' and car_state.energy < next_node[2]:
                    no_energy = True
                    break
            if no_energy:
                continue

            # Successful when there is enough energy to return to point P and all C and N have been delivered
            if car_state.transported_num == C_N_num:
                print("success")
                loop_flag = True
            if loop_flag:
                return car_state, loop_cnt
                break

            for next_info in car_state.current_node.adjacent_list:
                # Find the corresponding next_node
                next_node = None
                for node in self.node_list:
                    if node.x == next_info[0] and node.y == next_info[1]:
                        next_node = node
                        break
                

                if car_state.energy < next_info[2]:
                    continue
                
                h = heuristic_function_1 if self.heuristic_function_id == 1 else heuristic_function_2

                if next_node.type == 'P':
                    heapq.heappush(open_set, (next_info[2]+h(car_state), CarState(50, car_state.C_num, car_state.N_num, 
                    next_node, car_state, car_state.transported_num), visited_c_n))

                elif next_node.type == 'C' and (next_node.x, next_node.y) not in visited_c_n:
                    heapq.heappush(open_set, (next_info[2]+h(car_state), CarState(car_state.energy-next_info[2], car_state.C_num+1, car_state.N_num, 
                    next_node, car_state, car_state.transported_num), visited_c_n + [(next_node.x, next_node.y)]))

                elif next_node.type == 'N' and (next_node.x, next_node.y) not in visited_c_n:
                    if car_state.transported_num == 10:
                        heapq.heappush(open_set, (0, CarState(car_state.energy-next_info[2], car_state.C_num, car_state.N_num+1, 
                        next_node, car_state, car_state.transported_num), visited_c_n + [(next_node.x, next_node.y)]))
                        
                    else:
                        heapq.heappush(open_set, (next_info[2]+h(car_state), CarState(car_state.energy-next_info[2], car_state.C_num, car_state.N_num+1, 
                        next_node, car_state, car_state.transported_num), visited_c_n + [(next_node.x, next_node.y)]))
                

                elif next_node.type == 'CC' and car_state.C_num > 0:
                    heapq.heappush(open_set, (-1, CarState(car_state.energy-next_info[2], 0, car_state.N_num, 
                    next_node, car_state, car_state.transported_num + car_state.C_num), visited_c_n))

                elif next_node.type == 'CN' and car_state.N_num > 0:
                    if car_state.transported_num == 10:
                        heapq.heappush(open_set, (0, CarState(car_state.energy-next_info[2], car_state.C_num, 0, 
                        next_node, car_state, car_state.transported_num + car_state.N_num), visited_c_n))

                    else:
                        heapq.heappush(open_set, (next_info[2]+h(car_state), CarState(car_state.energy-next_info[2], car_state.C_num, 0, 
                        next_node, car_state, car_state.transported_num + car_state.N_num), visited_c_n))
                


if __name__ == '__main__':

    start_time = time.time()

    heuristic_function_id = int(sys.argv[2])
    mp_df = pd.read_csv(sys.argv[1], header=None, sep=';')
    mp_list = mp_df.values.tolist()

    a_star = AStar(mp_list, heuristic_function_id)
    a_star.map_to_node()
    car_state, loop_cnt = a_star.a_star_for_tsp()

    end_time = time.time()
    used_energy = 1
    mp_path = []

    # set output
    with open(sys.argv[1][:-4] + '-' + sys.argv[2] + '.output', 'w') as f:
        # generate path
        node_path = []
        while car_state:
            node_path.append(car_state.current_node)
            car_state = car_state.pre_car

        # generate mp path through node path
        for i in range(1, len(node_path)):
            start_x = node_path[i-1].x
            start_y = node_path[i-1].y
            end_x = node_path[i].x
            end_y = node_path[i].y

            # bfs add full path
            _, path = bfs(start_x, start_y, end_x, end_y, mp_list)
            mp_path.extend(path[:-1])

        mp_path.append((7, 3))
        mp_path = mp_path[::-1]

        # Eventually you need to get back to point P
        _, back_p_path = bfs(mp_path[-1][0], mp_path[-1][1], 7, 3, mp_list)
        mp_path.extend(back_p_path[1:])

        # print(mp_path)

        energy = 50
        f.write('(8, 4):P:50\n')
        for i in range(1, len(mp_path)):
            pre_x, pre_y = mp_path[i-1]
            now_x, now_y = mp_path[i]

            if mp_list[now_x][now_y] in ('C', 'N', 'CC', 'CN', 'P'):
                used_energy += 1
            else:
                used_energy += int(mp_list[now_x][now_y])


            if mp_list[pre_x][pre_y] == 'P':
                energy = 50

            if mp_list[now_x][now_y] in ('C', 'N', 'CC', 'CN'):
                energy -= 1
            elif mp_list[now_x][now_y] == 'P':
                energy = 50
            else:
                energy -= int(mp_list[now_x][now_y])
            f.write('({}, {}):{}:{}\n'.format(mp_path[i][0]+1, mp_path[i][1]+1, mp_list[mp_path[i][0]][mp_path[i][1]], energy))
    
    with open(sys.argv[1][:-4] + '-' + sys.argv[2] + '.stat', 'w') as f:
        f.write('Tiempo total: {}\n'.format(end_time - start_time))
        f.write('Coste total: {}\n'.format(used_energy))
        f.write('Longitud del plan: {}\n'.format(len(mp_path)))
        f.write('Nodos expandidos: {}\n'.format(loop_cnt))



