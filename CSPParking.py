from constraint import *
import sys

m, n = 0, 0
pe_set = set()
car_list = []
C_set = set()
TSU_set = set()
TNU_set = set()
parking_list = []
car_all_name_dict = {}
problem = Problem()

def get_input():
    with open(sys.argv[1], 'r') as f:
        # read the first line and set m n
        line = f.readline()
        global m, n
        m, n = map(int, line.split('x'))

        # read the second line and set PE
        line = f.readline().strip()
        i = 0
        while i < len(line):
            if line[i].isdigit():
                # read a pair of (a,b)
                # a b maybe have more than one digit
                a, b = 0, 0
                while line[i].isdigit():
                    a = a * 10 + int(line[i])
                    i += 1
                i += 1
                while line[i].isdigit():
                    b = b * 10 + int(line[i])
                    i += 1
                pe_set.add(b + (a - 1) * n) # (a, b) -> (a-1) * n + b
            else:
                i += 1
        
        # read from the third line to the last second line
        for line in f.readlines():
            line = line.strip()
            # add car_id to car_list, car_id maybe have more than one digit
            car_id = 0
            i = 0
            while line[i].isdigit():
                car_id = car_id * 10 + int(line[i])
                i += 1
            car_list.append(car_id)
            car_all_name_dict[car_id] = line    

            if line[-1] == 'C':
                C_set.add(car_id)
            if 'S' in line:
                TSU_set.add(car_id)
            if 'N' in line:
                TNU_set.add(car_id)

def do_constraint():
    parking_list.extend([i for i in range(1, m * n + 1)])

    problem.addVariables(car_list, parking_list)

    # constraint 1 and 2
    problem.addConstraint(AllDifferentConstraint())

    # constraint 3
    for car in car_list:
        if car in C_set:
            problem.addConstraint(InSetConstraint(pe_set), [car])

    # helper function for compute row and col
    def compute_row_col(parking):
        row = parking // n + 1
        col = parking % n
        if col == 0:
            col = n
            row -= 1
        return row, col

    # constraint 4
    def check_constraint_4(some_tsu, some_tun):
        tsu_row, tsu_col = compute_row_col(some_tsu)
        tun_row, tun_col = compute_row_col(some_tun)
        if tsu_row == tun_row and tun_col > tsu_col:
            return False
        return True

    for a in TSU_set:
        for b in TNU_set:
            problem.addConstraint(check_constraint_4, [a, b])

    # constraint 5
    def check_constraint_5(a, b, c):
        a_row, a_col = compute_row_col(a)
        b_row, b_col = compute_row_col(b)
        c_row, c_col = compute_row_col(c)

        # calculate adjacent parking spaces
        a_left_row, a_left_col = a_row - 1, a_col
        a_right_row, a_right_col = a_row + 1, a_col

        flag_left, flag_right = True, True
        if a_left_row == 0:
            flag_left = False
        if a_right_row == m + 1:
            flag_right = False
        if a_left_row == b_row and a_left_col == b_col:
            flag_left = False
        if a_right_row == c_row and a_right_col == c_col:
            flag_right = False

        if flag_left == False and flag_right == False:
            return False
        return True

    for a in car_list:
        for b in car_list:
            for c in car_list:
                if a != b and a != c and b != c:
                    problem.addConstraint(check_constraint_5, [a, b, c])



def set_output():
    res = problem.getSolutions()
    with open(sys.argv[1] + '.csv', 'w') as f:
        f.write('"N. Sol:",' + str(len(res)) + '\n')
        for i in range(min(3, len(res))):
            tmp_dict_parking_to_car = {}
            for car_id in res[i]:
                tmp_dict_parking_to_car[res[i][car_id]] = car_all_name_dict[car_id]
            for j in range(1, m * n + 1):
                if j in tmp_dict_parking_to_car:
                    f.write('"' + tmp_dict_parking_to_car[j] + '"')
                else:
                    f.write('"-"')
                if j % n == 0:
                    f.write('\n')
                else:
                    f.write(',')
            f.write('\n')
                


if __name__ == '__main__':
    get_input()
    do_constraint()
    set_output()
