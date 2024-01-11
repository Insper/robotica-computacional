import numpy as np
import matplotlib.pyplot as plt

class Mapa:
    def __init__(self, linhas=30, colunas=7):
        """
        Inicializa a grade com as dimensões especificadas e posiciona o carro.
        """
        self.linhas = linhas
        self.colunas = colunas
        # self.grade = np.ones((linhas, colunas))  # Inicializa a grade com zeros (espaço livre)
        np.random.seed(0) # Para garantir a reprodutibilidade
        self.grade = np.random.choice([0, 1], size=(linhas, colunas), p=[0.9, 0.1]) * 2
        self.grade[-2:, :] = 0  # A última linha deve ser livre
        self.grade_init = self.grade.copy()
        # 10% should be walls
        print(f"Grade inicial:\n{self.grade}")

        self.posicao = (-1, 3)
        self.atualizar_posicao(self.posicao)

    def atualizar_posicao(self, nova_posicao):
        """
        Atualiza a posição do carro na grade.
        """
        if self.grade[nova_posicao] == 1:
            raise ValueError("Movimento inválido: encontrou uma parede.")
        # Limpa a posição antiga do carro
        self.grade = self.grade_init.copy()
        # Atualiza para a nova posição
        self.grade[nova_posicao] = 1
        self.posicao = nova_posicao

    def mostrar(self):
        """
        Exibe a grade atual.
        """
        plt.figure(figsize=(6, 10))
        plt.imshow(self.grade, cmap='hot', interpolation='nearest')
        plt.axis('off')
        plt.grid(True)
        plt.show()

def main():
    grid = Mapa()
    grid.mostrar()

if __name__=="__main__":
    main()