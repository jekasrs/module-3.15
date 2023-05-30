from Generator import Generator


def read_number_of_ids(size):
    ids = []
    for i in range(0, size):
        current_id = int(input("Enter next id: "))
        ids.append(current_id)
    print()
    return ids


def main():
    aruco_type_num = int(
        input("Choose aruco dictionary (number):\n1. DICT_4X4_50\n2. DICT_4X4_100\n3. DICT_4X4_250\n4. "
              "DICT_4X4_1000\ndictionary="))
    size = int(input("Enter number of markers: "))
    ids = read_number_of_ids(size)
    gen = Generator(aruco_type_num, ids)
    gen.generate_all()


if __name__ == '__main__':
    main()
