import pytest
import cv2
from unittest.mock import patch
import numpy as np
import traceback

def run_blank_1(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr, point = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex1(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr, point = RodaAtividade.run(bgr)
    return {'point':point}

def check_ex1(result):
        assert result['frame01']['point'][0] == pytest.approx(515., abs=20.), "Ponto de fuga em x esta fora do esperado para a imagem frame01.jpg"
        assert result['frame01']['point'][1] == pytest.approx(326., abs=20.), "Ponto de fuga em y esta fora do esperado para a imagem frame01.jpg"
        if result['blank'] is True:
            pass
        elif str(result['blank']['error']) == "list index out of range":
            assert False, "Seu codigo deve ser robusto a ausencia da pista"
        else:
            assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])
def test_ex1():
    from ex1 import LinhaBranca
    RodaAtividade = LinhaBranca()

    result = {
        'blank': run_blank_1(RodaAtividade),
        'frame01': run_ex1(RodaAtividade, 'img/frame01.jpg'),
    }
    print(result)
    check_ex1(result)

##############################################################################################################

def run_blank_2(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr, texto = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex2(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr, texto = RodaAtividade.run(bgr)
    return {'texto':texto}

def check_ex2(result):
        assert result['domino1']['texto'] == '2 por 6', "Valor do domino esta fora do esperado para a imagem domino.jpg"
        if result['blank'] is True:
            pass
        elif str(result['blank']['error']) == "list index out of range":
            assert False, "Seu codigo deve ser robusto a ausencia de circulo"
        else:
            assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])
def test_ex2():
    from ex2 import ClassificaDominoes
    RodaAtividade = ClassificaDominoes()

    result = {
        'blank': run_blank_2(RodaAtividade),
        'domino1': run_ex2(RodaAtividade, 'img/domino.jpg'),
    }
    print(result)
    check_ex2(result)

##############################################################################################################

def run_blank_3(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr, ranked_arucos, closest_aruco = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex3(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr, ranked_arucos, closest_aruco = RodaAtividade.run(bgr)
    return {'closest_aruco':closest_aruco,'ranked_arucos':ranked_arucos}

def check_ex3(result):
        assert result['aruco']['closest_aruco'][0] == 'green', "Cor do aruco mais proximo esta fora do esperado para a imagem aruco.jpg"
        assert result['aruco']['closest_aruco'][1] == 11, "ID do aruco mais proximo esta fora do esperado para a imagem aruco.jpg"
        assert result['aruco2']['closest_aruco'][0] == 'green', "Cor do aruco mais proximo esta fora do esperado para a imagem aruco.jpg"
        assert result['aruco2']['closest_aruco'][1] == 11, "ID do aruco mais proximo esta fora do esperado para a imagem aruco.jpg"
        if result['blank'] is True:
            pass
        elif str(result['blank']['error']) == "list index out of range":
            assert False, "Seu codigo deve ser robusto a ausencia dos creepers"
        else:
            assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])
def test_ex3():
    from ex3 import DistanceEstimator
    RodaAtividade = DistanceEstimator()

    result = {
        'blank': run_blank_3(RodaAtividade),
        'aruco': run_ex3(RodaAtividade, 'fig/aruco.jpg'),
        'aruco2': run_ex3(RodaAtividade, 'fig/aruco2.jpg'),
    }
    print(result)
    check_ex3(result)

##############################################################################################################

def run_blank_4(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex4(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr = RodaAtividade.run(bgr)
    return RodaAtividade

def check_ex4(result):
        assert len(result['run']['dogA']) == 1, "Tracking do cachorro A esta fora do esperado para a sequencia (dogs1.png,dogs2.png)"
        assert len(result['run']['dogB']) == 2, "Tracking do cachorro B esta fora do esperado para a sequencia (dogs1.png,dogs2.png)"
        assert result['dogB'] == pytest.approx(5.22, abs=1.), "Deslocamento do cachorro B esta fora do esperado para a sequencia (dogs1.png,dogs2.png)"
        if result['blank'] is True:
            pass
        elif str(result['blank']['error']) == "list index out of range":
            assert False, "Seu codigo deve ser robusto a ausencia de circulo"
        else:
            assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])
def test_ex4():
    from ex4 import DogTracker
    RodaAtividade = DogTracker()
    result = {
        'blank': run_blank_4(RodaAtividade),
    }
    RodaAtividade = run_ex4(RodaAtividade, 'img/dogs1.png')
    RodaAtividade = run_ex4(RodaAtividade, 'img/dogs2.png')
    result['run'] = RodaAtividade.tracking
    center1 = RodaAtividade.bbox_center(result['run']['dogB'][0])
    center2 = RodaAtividade.bbox_center(result['run']['dogB'][1])
    result['dogB'] = RodaAtividade.bbox_distance(center1, center2)

    print(result)
    check_ex4(result)