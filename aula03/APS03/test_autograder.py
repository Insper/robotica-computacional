import cv2
import biblioteca
import biblioteca2
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
img = None

## PARTE 1 - SEGMENTAR LINHA AMARELA        
def test_segmenta_amarela():
    try:
        global img
        global mask
        img = cv2.imread("img/frame01.jpg")
        mask = biblioteca.segmenta_linha_amarela(img)
    except Exception as e:
        print('PARTE 1 - Falha na função segmenta_linha_amarela.')
        print(str(e))

    assert mask[620,676] == 255, "PARTE 1 - segmentacao esta fora do esperado"



## PARTE 2 - CENTRO LINHA AMARELA
def test_centro_amarelo():
    try:
        global X
        global Y
        contornos = biblioteca.encontrar_contornos(mask)
        cv2.drawContours(img, contornos, -1, [0, 0, 255], 2)
        _, X, Y = biblioteca.encontrar_centro_dos_contornos(img, contornos)
        
    except Exception as e:
        print('PARTE 2 - Falha na função encontrar_centro_dos_contornos.')
        print(str(e))

    resp = [676, 620]
    X = np.array(X)
    Y = np.array(Y)
    assert np.any(np.logical_and(resp[0] * 0.9 <= X, X <= resp[0] * 1.1)) and np.any(np.logical_and(resp[1] * 0.9 <= Y, Y <= resp[1] * 1.1)), "PARTE 2 - centro esta fora do esperado"


## PARTE 3 - REGRESSAO
def test_regressao():
    try:
        global lm
        resp = [-7.560509554140145, 5722.83057324842]
        _, lm = biblioteca.regressao_por_centro(img, X,Y)
        
    except Exception as e:
        print('PARTE 3 - Falha na função regressao_por_centro.')
        print(str(e))

    #assert resp[0] * 0.9 <= lm.coef_ <= resp[0] * 1.1 and resp[1] * 0.9 <= lm.intercept_ <= resp[1] * 1.1, "PARTE 3 - regressao esta fora do esperado"


## PARTE 4 - ANGULO
def test_angulo():
    try:
        global angulo
        resp = 7.534561718846401
        angulo = biblioteca.calcular_angulo_com_vertical(img, lm)
       
    except  Exception as e:
        print('PARTE 4 - Falha na função calcular_angulo_com_vertical.')
        print(str(e))

    assert resp * 0.9 <= abs(angulo) <= resp * 1.1, "PARTE 4 - angulo esta fora do esperado"

## PARTE 5 - VIDEO

## PARTE 6 - PONTO DE FUGA

## PARTE 6.1 - FAIXA BRANCA
def test_faixa_branca():
    try:
        global mask
        global img
        img = img.copy()[30:-30,200:-200] # Remover barra de tarefa da imagem
        mask = biblioteca2.segmenta_linha_branca(img)
        
    except Exception as e:
        print("PARTE 6.1 - Falha na função segmenta_linha_branca.")
        print(str(e))
    
    assert mask[400,690] == 255, "PARTE 6.1 - Reta nao esta bem segmentada"

## PARTE 6.2 - LINHA
def test_linha():
    try:
        global linhas
        linhas = biblioteca2.estimar_linha_nas_faixas(img, mask)
    except Exception as e:
        print("PARTE 6.2 - Falha na função calcular_equacao_das_retas.")
        print(str(e))

## PARTE 6.2 - EQUACAO
def test_equacao():
    try:
        global equacoes
        resp = [[-0.3643410852713174, 519.0], [0.38152610441767054, 131.63855421686765]]
        equacoes = biblioteca2.calcular_equacao_das_retas(linhas)

    except Exception as e:
        print("PARTE 6.2 - Falha na função calcular_equacao_das_retas.")
        print(str(e))

    for i,(m, h) in enumerate(equacoes):
        if m < 0:
            assert abs(resp[0][0]) * 0.9 <= abs(m) <= abs(resp[0][0]) * 1.1 and abs(resp[0][1]) * 0.9 <= abs(h) <= abs(resp[0][1]) * 1.1, f"PARTE 6.2 - equacao da reta {m}, {h} esta fora do esperado"
        else :
            assert abs(resp[1][0]) * 0.9 <= abs(m) <= abs(resp[1][0]) * 1.1 and abs(resp[1][1]) * 0.9 <= abs(h) <= abs(resp[1][1]) * 1.1, f"PARTE 6.2 - equacao da reta {m}, {h} esta fora do esperado"



## PARTE 6.3 - PF
def test_pf():
    try:
        resp = (519, 329)
        _, pontof = biblioteca2.calcular_ponto_de_fuga(img, equacoes)

    except Exception as e:
        print("PARTE 6.3 - Falha na função calcular_ponto_de_fuga.")
        print(str(e))

    assert resp[0] * 0.9 <= pontof[0] <= resp[0] * 1.1 and resp[1] * 0.9 <= pontof[1] <= resp[1] * 1.1, "PARTE 6.3 - ponto de fuga esta fora do esperado"

