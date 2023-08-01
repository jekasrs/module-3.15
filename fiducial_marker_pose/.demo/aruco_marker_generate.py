import cv2
import sys
import yaml


class Generator:
    ARUCO_DICTIONARIES = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    }

    def __init__(self, path_to_config):
        config = open(path_to_config, 'r')
        data = yaml.full_load(config)

        self.__aruco_type = data.get('aruco_type')
        self.__ids = data.get('marker_ids')
        self.__image_size = data.get('image_size')
        self.__marker_border = data.get('marker_border')
        self.__dir_path = data.get('dir_path')
        self.__aruco_dict = \
            cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTIONARIES[self.__aruco_type])

    def generate_all(self):
        for marker_id in self.__ids:
            image = cv2.aruco.generateImageMarker(self.__aruco_dict, marker_id,
                                                  self.__image_size, self.__marker_border)

            name = self.__dir_path + str.lower(self.__aruco_type) + '_' + str(marker_id) + '.png'
            cv2.imwrite(name, image)

            print(f'Marker with type {self.__aruco_type} and id {marker_id} is saved')


def main():
    if len(sys.argv) == 1:
        path_to_config = 'aruco_marker_generate.yaml'
        gen = Generator(path_to_config)
        gen.generate_all()
    else:
        option = sys.argv[1]
        path_to_config = sys.argv[2]
        if option == '--config-dir':
            gen = Generator(path_to_config)
            gen.generate_all()
        else:
            print(f'Unknown option: {option}')


if __name__ == '__main__':
    main()
