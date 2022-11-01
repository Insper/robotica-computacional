import pytest
import cv2
import matplotlib.pyplot as plt
import fotogrametria
import importlib
from webcam import calcular_angulo_e_distancia_na_image_da_webcam


## PARTE 1 - FOCO
def test_foco:
    try:
        resp = 629.9212598425197
        assert resp * 0.9 <= fotogrametria.encontrar_foco(80,12.70,100) <= resp * 1.1, "PARTE 1 - foco resultante está incorreto"
    except Exception as e:
        print("PARTE 1 - Falha na função encontrar_foco")
        pytest.fail(e, pytrace=True)

## PARTE 2 - SEGMENTAR
def test_segmentar:    
    try:
        img = cv2.imread("notebook_aux/folha_atividade.png")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    except Exception as e:
        print('PARTE 2 - Falha ao Abrir a Imagem de Teste')
        pytest.fail(e, pytrace=True)
    else:
        try:
            segmentado_ciano = fotogrametria.segmenta_circulo_ciano(hsv)
            segmentado_magenta = fotogrametria.segmenta_circulo_magenta(hsv)
        except Exception as e:
            print('PARTE 2 - Falha na função de segmentar o circulo')
            pytest.fail(e, pytrace=True)

## PARTE 3 - CONTORNO
def test_contour:
    try:
        # Area Ciano: 591888.0 - Area Magenta: 591835.0
        rc = 591888.0 
        rm = 591835.0

        ccontorno_ciano = fotogrametria.encontrar_maior_contorno(segmentado_ciano)
        contorno_magenta = fotogrametria.encontrar_maior_contorno(segmentado_magenta)

        assert rc * 0.9 <= cv2.contourArea(ccontorno_ciano) <= rc * 1.1, "PARTE 3 - contorno_ciano esta incorreto"
        assert rm * 0.9 <= cv2.contourArea(contorno_magenta) <= rm * 1.1, "PARTE 3 - contorno_magenta esta incorreto"
    except Exception as e:
        print("PARTE 3 - Falha na função de encontrar contorno")
        pytest.fail(e, pytrace=True)

## PARTE 4 - CENTRO
def test_centro:
    try:
        # Centro Ciano: (1079, 2802) - Centro Magenta: (1080, 816)
        centro_ciano = fotogrametria.encontrar_centro_contorno(ccontorno_ciano)
        centro_magenta = fotogrametria.encontrar_centro_contorno(contorno_magenta)
        img_4 = cv2.circle(img,centro_ciano, 10, (255,255,255), -1)
        img_4 = cv2.circle(img_4,centro_magenta, 10, (255,255,255), -1)

        x,y = centro_ciano
        assert min(img_4[y][x] == [255,255,255]), "PARTE 4 - centro_ciano esta incorreto"
        x,y = centro_magenta
        assert min(img_4[y][x] == [255,255,255]), "PARTE 4 - centro_magenta esta incorreto"
    except Exception as e:
        print("PARTE 4 - Falha ao encontrar o centro do contorno")
        pytest.fail(e, pytrace=True)

## PARTE 5 - FOCO
def test_outrofoco:
    try:
        # h = 1986.0002517623204 - f = 1014.1732283464568
        rh = 1986.0002517623204
        rf = 1014.1732283464568
        h = fotogrametria.calcular_h(centro_ciano, centro_magenta)
        f = fotogrametria.encontrar_foco(80,12.70,161.0)

        assert rh * 0.9 <= h <= rh * 1.1, "PARTE 5 - h esta incorreto"
        assert rf * 0.9 <= f <= rf * 1.1, "PARTE 5 - f esta incorreto"
    except Exception as e:
        print('PARTE 5 - Falha ao calcular a distancia entre os centros dos circulos')
        pytest.fail(e, pytrace=True)

## PARTE 6 - DISTANCIA
def test_distance:
    try:
        img_test = cv2.imread("img/test01.jpg")
    except Exception as e:
        print('PARTE 6 - Falha ao carregar a imagem de teste01.jpg')
        pytest.fail(e, pytrace=True)
    else:
        try:
            # d = 40.124415891157206
            dr = 40.124415891157206
            h, centro_ciano, centro_magenta, contornos_img = fotogrametria.calcular_distancia_entre_circulos(img_test)
            d = fotogrametria.encontrar_distancia(f,12.70,h)
            assert dr * 0.9 <= d <= dr * 1.1, "PARTE 6 - Incorreto"
            
        except Exception as e:
            print('PARTE 6 - Falha ao calcular a distancia ate os circulos')
            pytest.fail(e, pytrace=True)

## PARTE 7 - ANGULO
def test_angle:
    try:
        img_test = cv2.imread("img/angulo04.jpg")
    except Exception as e:
        print('PARTE 7 - Falha ao carregar a imagem de teste01.jpg')
        pytest.fail(e, pytrace=True)
    else:
        try:
            # angulo = 28.56758430890487
            ra = 28.56758430890487

            h, centro_ciano, centro_magenta, contornos_img = fotogrametria.calcular_distancia_entre_circulos(img_test)
            d = fotogrametria.encontrar_distancia(f,12.70,h)
            angulo = fotogrametria.calcular_angulo_com_horizontal_da_imagem(centro_ciano, centro_magenta)
            assert ra * 0.9 <= angulo <= ra * 1.1, "PARTE 7 - Inorreto"
        except Exception as e:
            print('PARTE 7 - Falha ao calcular o angulo entre os circulos')
            pytest.fail(e, pytrace=True)

## PARTE 8 - WEBCAM
def webcam_test:
    try:
        # distancia = 40.25589729191707 - angulo = 28.56758430890487
        img, distancia, angulo = calcular_angulo_e_distancia_na_image_da_webcam(img_test, 1014.1732283464568)
        assert ra * 0.9 <= angulo <= ra * 1.1, "PARTE 8 - ângulo incorreto"
        assert dr * 0.9 <= d <= dr * 1.1, "PARTE 8 - distância incorreta"
    except Exception as e:
        print('PARTE 8 - Falha ao calcular o angulo entre os circulos')
        pytest.fail(e, pytrace=True)
