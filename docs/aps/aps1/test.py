import cv2
import numpy as np
from ex1 import equaliza
from ex2 import realca_caixa_vermelha
from ex3 import recorta_leopardo
from ex4 import antartida, canada
from ex5 import realiza_diferencas
from ex6 import substitui_x_por_cinza
import time
import ast

delta_1 = 0
delta_2 = 0
delta_3 = 0
delta_4 = 0
delta_5 = 0
delta_6 = 0

def test_check_equalizeHist_usage():
    with open('ex1.py', 'r') as f:
        node = ast.parse(f.read())
        visitor = EqualizeHistVisitor()
        visitor.visit(node)
        assert not visitor.found, "Neste exercício não utilize a função cv2.equalizeHist()!"

class EqualizeHistVisitor(ast.NodeVisitor):
    def __init__(self):
        self.found = False

    def visit_Attribute(self, node):
        if isinstance(node.value, ast.Name) and node.value.id == 'cv2' and node.attr == 'equalizeHist':
            self.found = True
        self.generic_visit(node)

# courtesy of: https://pyimagesearch.com/2014/09/15/python-compare-two-images/
def mse(imageA, imageB):
    # the 'Mean Squared Error' between the two images is the
    # sum of the squared difference between the two images;
    # NOTE: the two images must have the same dimension
    err = np.sum((imageA.astype("float") - imageB.astype("float")) ** 2)
    err /= float(imageA.shape[0] * imageA.shape[1])

    # return the MSE, the lower the error, the more "similar"
    # the two images are
    return err

def ex1_equalization(img_submitted):
    assert img_submitted.min() == 0
    assert img_submitted.max() >= 254

def ex1_compare_img(img_submitted):
    img_expected = cv2.imread("expected_img/ex1.jpg", cv2.IMREAD_GRAYSCALE)
    err = mse(img_submitted, img_expected)
    assert err < 10, "Imagem está muito diferente do esperado"

def run_ex1():
    global delta_1
    start = time.perf_counter()
    img_given = cv2.imread("img/RinTinTin.jpg", cv2.IMREAD_GRAYSCALE)
    img_submitted = equaliza(img_given)
    delta_1 = time.perf_counter() - start
    return img_submitted
def test_ex1():
    img_submitted = run_ex1()

    assert f'{np.mean(img_submitted):.3f}' != '33.637', "Não fez"
    test_check_equalizeHist_usage()
    ex1_equalization(img_submitted)
    ex1_compare_img(img_submitted)

def run_ex2():
    global delta_2
    start = time.perf_counter()
    img_given = cv2.imread("img/ex2.jpg")
    img_submitted = realca_caixa_vermelha(img_given)
    delta_2 = time.perf_counter() - start
    return img_submitted, img_given
def test_ex2():
    img_submitted, img_given = run_ex2()

    assert f'{np.mean(img_submitted):.3f}' != '154.243', "Não fez"
    assert img_submitted.shape != img_given.shape, "A saída deve ser uma imagem binária"
    
    try:
        img_submitted = img_submitted[:, :, ::-1]
    except:
        img_submitted = img_submitted[:, ::-1]
    img_expected = cv2.imread("expected_img/ex2.jpg", cv2.IMREAD_GRAYSCALE)
    inter = cv2.bitwise_and(img_expected, img_submitted)

    area_esperada = np.sum(img_expected)
    area_inter = np.sum(inter)
    r = area_esperada / area_inter
    assert r >= 0.8 and r <= 1.5, "Área da caixa vermelha está fora do esperado"


def compara_imagens_alinhando(img_expected, img):
    tolerancia_w = int(img_expected.shape[1] / 20)
    tolerancia_h = int(img_expected.shape[0] / 20)
    img_expected = img_expected[
        tolerancia_h:-tolerancia_h, tolerancia_w:-tolerancia_w, :
    ]
    res = cv2.matchTemplate(img, img_expected, cv2.TM_SQDIFF_NORMED)
    min_val, max_val, minloc, maxloc = cv2.minMaxLoc(res)
    assert min_val <= 0.05, "Imagem está muito diferente do esperado"

def compara_dimensoes(img_expected, img, text=""):
    a = np.array(img.shape) / np.array(img_expected.shape)
    # check if the image is not too much bigger or smaller than expected
    assert np.all(a >= 0.9) and np.all(a <= 1.1), f"Dimensões da imagem {text}estão fora do esperado"

def run_ex3():
    global delta_3
    start = time.perf_counter()
    img_given = cv2.imread("img/ex3.png")
    img_submitted = recorta_leopardo(img_given)
    delta_3 = time.perf_counter() - start
    return img_submitted
def test_ex3():
    img_submitted = run_ex3()

    assert f'{np.mean(img_submitted):.3f}' != '137.358', "Não fez"

    img_expected = cv2.imread("expected_img/ex3_case1.png")
    compara_imagens_alinhando(img_expected, img_submitted)
    compara_dimensoes(img_expected, img_submitted)


def run_ex4():
    global delta_4
    start = time.perf_counter()
    img_given = cv2.imread("img/ex4.png")
    img_submitted1 = antartida(img_given)
    img_submitted2 = canada(img_given)
    delta_4 = time.perf_counter() - start
    return img_submitted1, img_submitted2
def test_ex4():
    img_submitted1, img_submitted2 = run_ex4()

    assert f'{np.mean(img_submitted1):.3f}' != '39.153', "Não fez a bandeira da Antártida"
    assert f'{np.mean(img_submitted2):.3f}' != '39.153', "Não fez a bandeira do Canadá"

    img_expected1 = cv2.imread("expected_img/ex4_ant.png")
    img_expected2 = cv2.imread("expected_img/ex4_can.png")
    compara_imagens_alinhando(img_expected1, img_submitted1)
    compara_dimensoes(img_expected1, img_submitted1, text="da bandeira da Antártida ")
    compara_imagens_alinhando(img_expected2, img_submitted2)
    compara_dimensoes(img_expected2, img_submitted2, text="da bandeira do Canadá ")

def ex5_compare_img(img_submitted, img_expected):
    return mse(img_submitted, img_expected)

def run_ex5():
    global delta_5
    start = time.perf_counter()
    img_given = cv2.imread("img/ex5.png")
    img_submitted = realiza_diferencas(img_given)
    delta_5 = time.perf_counter() - start
    return img_submitted
def test_ex5():
    img_submitted = run_ex5()
    
    assert f'{np.mean(img_submitted):.3f}' != '126.082', "Não fez"
    assert np.mean(img_submitted) < 50, "A imagem resultante está saturada! Dica: converta para int32 antes de fazer os cálculos e então converta para int8."

    img_expected_h = cv2.imread("expected_img/ex5h.png",0)
    img_expected_v = cv2.imread("expected_img/ex5v.png",0)

    err_h = ex5_compare_img(img_submitted, img_expected_h)
    err_v = ex5_compare_img(img_submitted, img_expected_v)
    
    assert err_h < 100 or err_v < 100, "A imagem resultante está muito diferente do esperado"
    assert err_v > err_h, "Parece que você calculou a diferença entre os pixels na vertical e não na horizontal"


def ex6_compare_img(img_submitted, img_expected):
    return mse(img_submitted, img_expected)

def run_ex6():
    global delta_6
    start = time.perf_counter()
    img_given = cv2.imread("img/ex6.png",0)
    img_submitted = substitui_x_por_cinza(img_given)
    delta_6 = time.perf_counter() - start
    return img_submitted
def test_ex6():
    img_submitted = run_ex6()

    assert f'{np.mean(img_submitted):.3f}' != '3.559', "Não fez"

    img_expected = cv2.imread("expected_img/ex6.png",0)
    err = ex6_compare_img(img_submitted, img_expected)
    assert err < 5, "A imagem resultante está muito diferente do esperado. Verifique se a saída está como o esperado."


def test_time_challenge():
    run_ex1()
    run_ex2()
    run_ex3()
    run_ex4()
    run_ex5()
    run_ex6()

    assert delta_1 < 0.01, "Tempo de execução do desafio 1 está muito alto, tente uma forma de otimizar seu código - Dica: Cheque os exercícios da atividade 2 do módulo 1." 
    assert delta_2 < 0.05, "Tempo de execução do desafio 2 está muito alto, tente uma forma de otimizar seu código"
    assert delta_3 < 0.05, "Tempo de execução do desafio 3 está muito alto, tente uma forma de otimizar seu código"
    assert delta_4 < 0.05, "Tempo de execução do desafio 4 está muito alto, tente uma forma de otimizar seu código"
    assert delta_5 < 0.05, "Tempo de execução do desafio 5 está muito alto, tente uma forma de otimizar seu código - Dica: O numpy pode subtrair arrays diretamente, ex: direita - esquerda."
    assert delta_6 < 0.05, "Tempo de execução do desafio 6 está muito alto, tente uma forma de otimizar seu código - Dica: Você só precisa olhar para os pixels que são iguais a 255."