import pytest
import cv2
from unittest.mock import patch
from ex1 import Atividade1
from ex2 import Atividade2
import numpy as np
import traceback

def run_blank_1(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr, D, angulo, h, f = RodaAtividade.calibration(bgr, D=80, H=12.5)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex1(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr, D, angulo, h, f = RodaAtividade.calibration(bgr, D=80, H=12.5)
    return {'f':f,'angulo':angulo,'h':h}

def check_ex1(result):
    assert result['angulo01']['f'] == pytest.approx(2054., abs=10.), "O foco da camera esta fora do esperado"
    assert result['angulo01']['h'] == pytest.approx(320., abs=2.), "A distancia entre os circulos esta fora do esperado"
    assert result['angulo01']['angulo'] == pytest.approx(-0.17849094755360373, abs=0.1), "O angulo, para a imagem angulo01.jpg, esta fora do esperado"
    assert result['angulo02']['angulo'] == pytest.approx(-51.98362315755636, abs=0.1), "O angulo, para a imagem angulo02.jpg, esta fora do esperado"
    assert result['angulo03']['angulo'] == pytest.approx(-88.92917554521304, abs=0.1), "O angulo, para a imagem angulo03.jpg, esta fora do esperado"
    assert result['angulo04']['angulo'] == pytest.approx(118.56758430890487, abs=0.1), "O angulo, para a imagem angulo04.jpg, esta fora do esperado"
    if result['blank'] is True:
        pass
    elif str(result['blank']['error']) == "list index out of range":
        assert False, "Seu codigo deve retornar -1 para D, angulo e h quando nao encontrar os circulos"
    else:
        assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])
def test_ex1():
    RodaAtividade = Atividade1()

    result = {
        'blank': run_blank_1(RodaAtividade),
        'angulo01': run_ex1(RodaAtividade, "img/angulo01.jpg"),
        'angulo02': run_ex1(RodaAtividade, "img/angulo02.jpg"),
        'angulo03': run_ex1(RodaAtividade, "img/angulo03.jpg"),
        'angulo04': run_ex1(RodaAtividade, "img/angulo04.jpg"),
    }
    print(result)
    check_ex1(result)


def run_blank_2(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    with patch('cv2.imshow'):
        try:
            bgr, m, h, xr, yr = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_single_contour(RodaAtividade):
    bgr = np.zeros((400,400,3), dtype=np.uint8)
    bgr = cv2.circle(bgr, (200, 200), 100, (0, 255, 255), -1)
    with patch('cv2.imshow'):
        try:
            bgr, m, h, xr, yr = RodaAtividade.run(bgr)
        except Exception as e:
            return {'error': e, 'traceback': traceback.format_exc().splitlines()[-2:-1]}
        
    return True
def run_ex2(RodaAtividade, fname):
    bgr = cv2.imread(fname)
    with patch('cv2.imshow'):
        bgr, m, h, xr, yr = RodaAtividade.run(bgr)
    return {'angular':m,'linear':h,'lenght':len(xr)}

def check_ex2(result):
    assert result['frame01']['angular'] == pytest.approx(-.4, abs=.2), "O foco da camera esta fora do esperado"
    assert result['frame01']['lenght'] == pytest.approx(3, abs=1), "O foco da camera esta fora do esperado"
    assert result['frame02']['angular'] == pytest.approx(.14, abs=.1), "O foco da camera esta fora do esperado"
    assert result['frame02']['lenght'] == pytest.approx(3, abs=1), "O foco da camera esta fora do esperado"
    assert result['frame03']['angular'] == pytest.approx(-.38, abs=.1), "O foco da camera esta fora do esperado"
    assert result['frame03']['lenght'] == pytest.approx(3, abs=1), "O foco da camera esta fora do esperado"
    assert result['frame04']['angular'] == pytest.approx(.42, abs=.1), "O foco da camera esta fora do esperado"
    assert result['frame04']['lenght'] == pytest.approx(3, abs=1), "O foco da camera esta fora do esperado"

    if result['blank'] is True:
        pass
    elif str(result['blank']['error']) == "list index out of range":
        assert False, "Seu codigo deve retornar -1 para D, angulo e h quando nao encontrar os circulos"
    else:
        assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem em branco - {0} - {1}".format(result['blank']['traceback'], result['blank']['error'])

    if result['run_single_contour'] is True:
        pass
    elif str(result['run_single_contour']['error']) == "`min_samples` may not be larger than number of samples: n_samples = 1.":
        assert False, "O metodo RANSAC nao deve ser chamado quando houver apenas um contorno"
    else:
        assert False, "Ocorreu um erro inesperado ao rodar seu codigo com uma imagem com apenas um contorno - {0} - {1}".format(result['run_single_contour']['traceback'], result['run_single_contour']['error'])
def test_ex2():
    RodaAtividade = Atividade2()

    result = {
        'blank': run_blank_2(RodaAtividade),
        'run_single_contour': run_single_contour(RodaAtividade),
        'frame01': run_ex2(RodaAtividade, "img/frame01.png"),
        'frame02': run_ex2(RodaAtividade, "img/frame02.png"),
        'frame03': run_ex2(RodaAtividade, "img/frame03.png"),
        'frame04': run_ex2(RodaAtividade, "img/frame04.png"),
    }
    check_ex2(result)
