import cv2
import matplotlib.pyplot as plt
import fotogrametria
import importlib
from webcam import calcular_angulo_e_distancia_na_image_da_webcam

output = {'result': 0}
## PARTE 1 - FOCO
try:
    resp = 629.9212598425197
    if resp * 0.9 <= fotogrametria.encontrar_foco(80,12.70,100) <= resp * 1.1 :
        output['result'] += 1
        output['1'] = "PARTE 1 - Correto"
    else:
        output['1'] = "PARTE 1 - Incorreto"
except:
    print("PARTE 1 - Falha na função encontrar_foco")
    output['1'] = "Erro: PARTE 1 - Falha na função encontrar_foco"


## PARTE 2 - SEGMENTAR
try:
    img = cv2.imread("notebook_aux/folha_atividade.png")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
except:
    print('PARTE 2 - Falha ao Abrir a Imagem de Teste')
else:
    try:
        segmentado_ciano = fotogrametria.segmenta_circulo_ciano(hsv)
        segmentado_magenta = fotogrametria.segmenta_circulo_magenta(hsv)
        output['result'] += 1
        output['2'] = "PARTE 2 - Correto"
    except:
        print('PARTE 2 - Falha na função de segmentar o circulo')
        output['2'] = "Erro: PARTE 2 - Falha na função de segmentar o circulo"

## PARTE 3 - CONTORNO
try:
    # Area Ciano: 591888.0 - Area Magenta: 591835.0
    rc = 591888.0 
    rm = 591835.0

    ccontorno_ciano = fotogrametria.encontrar_maior_contorno(segmentado_ciano)
    contorno_magenta = fotogrametria.encontrar_maior_contorno(segmentado_magenta)
    
    if rc * 0.9 <= cv2.contourArea(ccontorno_ciano) <= rc * 1.1:
        if rm * 0.9 <= cv2.contourArea(contorno_magenta) <= rm * 1.1:
            output['result'] += 1
            output['3'] = "PARTE 3 - Correto"
        else:
            output['3'] = "contorno_magenta esta incorreto"
    else:
        output['3'] = "contorno_ciano esta incorreto"
except:
    print("PARTE 3 - Falha na função de encontrar contorno")
    output['3'] = "Erro: PARTE 3 - Falha na função de encontrar contorno"

## PARTE 4 - CENTRO
try:
    # Centro Ciano: (1079, 2802) - Centro Magenta: (1080, 816)
    centro_ciano = fotogrametria.encontrar_centro_contorno(ccontorno_ciano)
    centro_magenta = fotogrametria.encontrar_centro_contorno(contorno_magenta)
    img_4 = cv2.circle(img,centro_ciano, 10, (255,255,255), -1)
    img_4 = cv2.circle(img_4,centro_magenta, 10, (255,255,255), -1)
    
    x,y = centro_ciano
    if min(img_4[y][x] == [255,255,255]):
        x,y = centro_magenta
        if min(img_4[y][x] == [255,255,255]):
            output['result'] += 1
            output['4'] = "PARTE 4 - Correto"
        else:
            output['4'] = "centro_magenta esta incorreto"
    else:
        output['4'] = "centro_ciano esta incorreto"
except:
    print("PARTE 4 - Falha ao encontrar o centro do contorno")
    output['4'] = "Erro: PARTE 4 - Falha ao encontrar o centro do contorno"

## PARTE 5 - FOCO
try:
    # h = 1986.0002517623204 - f = 1014.1732283464568
    rh = 1986.0002517623204
    rf = 1014.1732283464568
    h = fotogrametria.calcular_h(centro_ciano, centro_magenta)
    f = fotogrametria.encontrar_foco(80,12.70,161.0)

    if rh * 0.9 <= h <= rh * 1.1:
        if rf * 0.9 <= f <= rf * 1.1:
            output['result'] += 1
            output['5'] = "PARTE 5 - Correto"
        else:
            output['5'] = "f esta incorreto"
    else:
        output['5'] = "h esta incorreto"
except:
    print('PARTE 5 - Falha ao calcular a distancia entre os centros dos circulos')
    output['5'] = "Erro: PARTE 5 - Falha ao calcular a distancia entre os centros dos circulos"

## PARTE 6 - DISTANCIA
try:
    img_test = cv2.imread("img/test01.jpg")
except:
    print('PARTE 6 - Falha ao carregar a imagem de teste01.jpg')
else:
    try:
        # d = 40.124415891157206
        dr = 40.124415891157206
        h, centro_ciano, centro_magenta, contornos_img = fotogrametria.calcular_distancia_entre_circulos(img_test)
        d = fotogrametria.encontrar_distancia(f,12.70,h)
        if dr * 0.9 <= d <= dr * 1.1:
            output['result'] += 1
            output['6'] = "PARTE 6 - Correto"
        else:
            output['6'] = "PARTE 6 - Incorreto"
    except:
        print('PARTE 6 - Falha ao calcular a distancia ate os circulos')
        output['6'] = "Erro: PARTE 6 - Falha ao calcular a distancia ate os circulos"

## PARTE 7 - ANGULO
try:
    img_test = cv2.imread("img/angulo04.jpg")
except:
    print('PARTE 7 - Falha ao carregar a imagem de teste01.jpg')
else:
    try:
        # angulo = 28.56758430890487
        ra = 28.56758430890487

        h, centro_ciano, centro_magenta, contornos_img = fotogrametria.calcular_distancia_entre_circulos(img_test)
        d = fotogrametria.encontrar_distancia(f,12.70,h)
        angulo = fotogrametria.calcular_angulo_com_horinzontal_da_imagem(centro_ciano, centro_magenta)
        if ra * 0.9 <= angulo <= ra * 1.1:
            output['result'] += 1
            output['7'] = "PARTE 7 - Correto"
        else:
            output['7'] = "PARTE 7 - Incorreto"
    except:
        print('PARTE 7 - Falha ao calcular o angulo entre os circulos')
        output['7'] = "Erro: PARTE 7 - Falha ao calcular o angulo entre os circulos"

## PARTE 8 - WEBCAM
    try:
        # distancia = 40.25589729191707 - angulo = 28.56758430890487
        img, distancia, angulo = calcular_angulo_e_distancia_na_image_da_webcam(img_test, 1014.1732283464568)
        if ra * 0.9 <= angulo <= ra * 1.1:
            if dr * 0.9 <= d <= dr * 1.1:
                output['result'] += 1
                output['8'] = "PARTE 8 - Correto"
            else:
                output['8'] = "PARTE 8 - Incorreto"
        else:
            output['8'] = "PARTE 8 - Incorreto"
    except:
        print('PARTE 8 - Falha ao calcular o angulo entre os circulos')
        output['8'] = "Erro: PARTE 8 - Falha ao calcular o angulo entre os circulos"
