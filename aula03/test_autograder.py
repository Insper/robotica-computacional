import pytest
import cv2
import biblioteca
import biblioteca2
import biblioteca_cow
import numpy as np

mask = None
X = None
Y = None
lm = None
angulo = None
linhas = None
equacoes = None
resultados = None
animais = None

img = cv2.imread("img/frame01.jpg")

## PARTE 1 - SEGMENTAR LINHA AMARELA        
def segmenta_amarela():
    try:
        mask = biblioteca.segmenta_linha_amarela(img)
        assert mask[620,676] == 255, "PARTE 2 - segmentacao esta fora do esperado"
    except:
        print('PARTE 1 - Falha na função encontrar_centro_dos_contornos.')



## PARTE 2 - CENTRO LINHA AMARELA
def centro_amarelo():
    try:
        global X
        global Y
        contornos = biblioteca.encontrar_contornos(mask)
        cv2.drawContours(img, contornos, -1, [0, 0, 255], 2)
        _, X, Y = biblioteca.encontrar_centro_dos_contornos(img, contornos)
        resp = [676, 620]
        assert resp[0] * 0.9 <= X <= resp[0] * 1.1 and resp[1] * 0.9 <= Y <= resp[1] * 1.1, "PARTE 2 - centro esta fora do esperado"

    except Exception as e:
        print('PARTE 2 - Falha na função encontrar_centro_dos_contornos.')

## PARTE 3 - REGRESSAO
def regressao():
    try:
        global lm
        resp = [-7.560509554140145, 5722.83057324842]
        _, lm = biblioteca.regressao_por_centro(img, X,Y)
        assert resp[0] * 0.9 <= lm[0] <= resp[0] * 1.1 and resp[1] * 0.9 <= lm[1] <= resp[1] * 1.1, "PARTE 3 - regressao esta fora do esperado"

    except Exception as e:
        print('PARTE 3 - Falha na função regressao_por_centro.')

## PARTE 4 - ANGULO
def angulo():
    try:
        global angulo
        resp = 7.534561718846401
        _,angulo = biblioteca.calcular_angulo_com_vertical(img, lm)
        assert resp * 0.9 <= angulo <= resp * 1.1, "PARTE 4 - angulo esta fora do esperado"
    except  Exception as e:
        print('PARTE 4 - Falha na função calcular_angulo_com_vertical.')

## PARTE 5 - VIDEO

## PARTE 6 - PONTO DE FUGA
img = img.copy()[30:-30,200:-200] # Remover barra de tarefa da imagem

## PARTE 6.1 - FAIXA BRANCA
def faixa_branca():
    try:
        global mask
        mask = biblioteca2.segmenta_linha_branca(img)
        assert mask[405,690] == 255, "PARTE 6.1 - Reta nao esta bem segmentada"
    except Exception as e:
        print("PARTE 6.1 - Falha na função segmenta_linha_branca.")

## PARTE 6.2 - LINHA
def linha():
    try:
        global linhas
        _,linhas = biblioteca2.estimar_linha_nas_faixas(img, mask)
    except Exception as e:
        print("PARTE 6.2 - Falha na função calcular_equacao_das_retas.")

## PARTE 6.2 - EQUACAO
def equacao():
    try:
        global equacoes
        resp = [[-0.3643410852713174, 519.0], [0.38152610441767054, 131.63855421686765]]
        equacoes = biblioteca2.calcular_equacao_das_retas(linhas)
        for i,(m, h) in enumerate(equacoes):
            assert resp[i][0] * 0.9 <= m <= resp[i][0] * 1.1 and resp[i][1] * 0.9 <= h <= resp[i][1] * 1.1, "PARTE 6.2 - equacao da reta esta fora do esperado"

    except Exception as e:
        print("PARTE 6.2 - Falha na função calcular_equacao_das_retas.")


## PARTE 6.3 - PF
def pf():
    try:
        resp = (519, 329)
        _, pontof = biblioteca2.calcular_ponto_de_fuga(img, equacoes)
        assert resp[0] * 0.9 <= pontof[0] <= resp[0] * 1.1 and resp[1] * 0.9 <= pontof[1] <= resp[1] * 1.1, "PARTE 6.3 - ponto de fuga esta fora do esperado"

    except Exception as e:
        print("PARTE 6.3 - Falha na função calcular_ponto_de_fuga.")


## PARTE 7 - VACA
img = cv2.imread("cow_wolf/cow_wolf01.png")
    # Classes
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]
    # Carregar Rede
net = biblioteca_cow.load_mobilenet()
    # Detectar
CONFIDENCE = 0.7
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

## PARTE 7.1 - MOBILENET
def testmobilenet():
    try:
        global resultados
        _, resultados = biblioteca_cow.detect(net, img, CONFIDENCE, COLORS, CLASSES)
        resp = [('cow', 99.0637481212616, (379, 131), (560, 251)), ('horse', 94.41149830818176, (53, 103), (297, 286)), ('horse', 93.70213747024536, (626, 103), (860, 285))]
        assert resultados == resp, "PARTE 7.1 - Mobilenet não esta correta."
    
    except Exception as e:
        print("PARTE 7.1 - Falha na função detect.")

## PARTE 7.2 - CAIXAS
def caixas():
    try:
        global animais
        _, animais = biblioteca_cow.separar_caixa_entre_animais(img, resultados)
        assert animais == {'vaca': [[379, 131, 560, 251]], 'lobo': [53, 103, 860, 286]}, "PARTE 7.2 - Caixas não estão corretas"
    
    except Exception as e:
        print("PARTE 7.2 - Falha na função separar_caixa_entre_animais.")


## PARTE 7.3 - PERIGO
def perigo():
    try:
        for vaca in animais['vaca']:
            resp = 0.148124730951356
            assert resp * 0.9 <= biblioteca_cow.calcula_iou(vaca, animais['lobo']) <= resp * 1.1, "PARTE 7.3 - IoU não esta dentro do esperado"
    
    except Exception as e:
        print("PARTE 7.3 - Falha na função calcula_iou")
        # pytest.fail(e, pytrace=True)



