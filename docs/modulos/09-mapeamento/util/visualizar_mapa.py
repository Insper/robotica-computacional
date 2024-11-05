import matplotlib.pyplot as plt
import numpy as np
import cv2

def map_to_world( map_x, map_y, height, width, resolution, origin_x, origin_y):
    # Convert map coordinates (pixels) to world coordinates (meters)
    world_x = map_x * resolution + origin_x
    world_y = map_y * resolution + origin_y
    return world_x, world_y

def world_to_map( world_x, world_y, height, width, resolution, origin_x, origin_y):
    # Convert world coordinates (meters) to map coordinates (pixels)
    map_x = int((world_x - origin_x) / resolution)
    map_y = height - int((world_y - origin_y) / resolution)
    return map_y, map_x

def prep_map(map_path):
    """
    Prepara o mapa carregando e processando a imagem de entrada.

    Args:
        map_path (str): O caminho do arquivo do mapa.

    Returns:
        np.array: O mapa processado como um array numpy.
    """
    map_array = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    map_array[map_array == 0] = 0
    map_array[map_array == 205] = 128
    map_array[map_array == 254] = 255
    map_array[(map_array >= 60) & (map_array != 128) & (map_array != 255)] = 0
    map_array = map_array.astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    map_array = cv2.morphologyEx(map_array, cv2.MORPH_OPEN, kernel)
    map_array = np.flipud(map_array) # TODO - um deles precisa ser removido
    map_array = np.pad(map_array, ((0, 200), (0, 200)), 'constant', constant_values=128)
    return map_array

def plot_path(map, point):
    """
    Plota o mapa e o caminho encontrado, incluindo o caminho simplificado.

    Args:
        path (list): O caminho completo encontrado.
    """

    plt.figure(figsize=(10, 10))
    plt.imshow(map, cmap='gray')
    plt.scatter(point[0], point[1], color='green', s=100, label='Início')
   
    plt.title("Mapa - aperte 'q' para fechar e selecionar outro ponto")

    plt.legend()
    plt.axis('equal')
    plt.show()

def main():
    map = prep_map('/home/borg/Documents/robotica-computacional/docs/modulos/09-mapeamento/util/map.pgm')
    height, width = map.shape
    
    # open yaml file and get map metadata
    with open('/home/borg/Documents/robotica-computacional/docs/modulos/09-mapeamento/util/map.yaml', 'r') as file:
        for line in file:
            if 'resolution' in line:
                resolution = float(line.split(': ')[1])
            if 'origin' in line:
                line = line.replace('[', '').replace(']', '').replace('origin: ', '').replace('\n', '')
                line = line.split(', ')
                origin_x = float(line[0])
                origin_y = float(line[1])

    plot_path(map, (0,0))
    while True:
        point = input("Digite as coordenadas do ponto x,y do mapa, e.g 50,17: ")
        point = tuple([int(i) for i in point.split(',')])
        print(map_to_world(point[0], point[1], height, width, resolution, origin_x, origin_y))
        plot_path(map, point)


if __name__ == '__main__':
    main()