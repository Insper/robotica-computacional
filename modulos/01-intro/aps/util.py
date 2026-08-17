import numpy as np
import os

class Mapa:
    def __init__(self, linhas=30, colunas=7):
        """
        Inicializa a grade com as dimensões especificadas e posiciona o carro.
        """
        self.linhas = linhas
        self.colunas = colunas
        # np.random.seed(0) # !!! Remova o comentário para sempre gerar o mesmo mapa !!! 
        self.grade = np.random.choice([0, 1], size=(linhas, colunas), p=[0.9, 0.1]) * 2
        self.grade[-2:, :] = 0  # A última linha deve ser livre
        self.grade_init = self.grade.copy()

        self.posicao = (self.linhas - 1, 3)
        self.atualizar_posicao(self.posicao)

    def atualizar_posicao(self, nova_posicao):
        """
        Atualiza a posição do carro na grade.
        Levanta exceções se o movimento for inválido.
        """
        # Verifica se a nova posição está dentro dos limites
        if (nova_posicao[0] < 0 or nova_posicao[0] >= self.linhas or 
            nova_posicao[1] < 0 or nova_posicao[1] >= self.colunas):
            raise ValueError("GAME OVER! Movimento inválido: posição fora dos limites do mapa.")
        
        # Verifica se há obstáculo na posição original
        if self.grade_init[nova_posicao] == 2:
            raise ValueError("GAME OVER! Movimento inválido: encontrou um obstáculo.")
        
        # Limpa a posição antiga do carro
        self.grade = self.grade_init.copy()
        # Atualiza para a nova posição
        self.grade[nova_posicao] = 1
        self.posicao = nova_posicao

    def mostrar(self):
        """
        Exibe a grade atual no terminal usando strings.
        """
        # Limpa o terminal (funciona no Windows, Linux e macOS)
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # Mapa de Símbolos
        simbolos = {
            0: '  ',   # Espaço livre
            1: '🚗',   # Carro
            2: '██'    # Obstáculo/parede
        }
        
        # Alternativa com caracteres ASCII se os emojis não funcionarem
        # simbolos = {
        #     0: '  ',   # Espaço livre
        #     1: 'C ',   # Carro
        #     2: '##'    # Obstáculo/parede
        # }
        
        print("┌" + "─" * (self.colunas * 2) + "┐")
        
        for linha in self.grade:
            linha_str = "│"
            for celula in linha:
                linha_str += simbolos[celula]
            linha_str += "│"
            print(linha_str)
        
        print("└" + "─" * (self.colunas * 2) + "┘")
        print(f"Posição do carro: linha {self.posicao[0]}, coluna {self.posicao[1]}")

    def mover_carro(self, direcao):
        """
        Move o carro na direção especificada.
        Direções: 'cima', 'baixo', 'esquerda', 'direita'
        Retorna: True se movimento válido, False se deve encerrar o jogo
        """
        linha_atual, coluna_atual = self.posicao
        
        movimentos = {
            'cima': (-1, 0),
            'baixo': (1, 0),
            'esquerda': (0, -1),
            'direita': (0, 1)
        }
        
        if direcao not in movimentos:
            print("Direção inválida! Use: 'cima', 'baixo', 'esquerda', 'direita'")
            return True  # Não encerra o jogo por comando inválido
        
        delta_linha, delta_coluna = movimentos[direcao]
        nova_linha = linha_atual + delta_linha
        nova_coluna = coluna_atual + delta_coluna
        nova_posicao = (nova_linha, nova_coluna)
        
        try:
            self.atualizar_posicao(nova_posicao)
            return True  # Movimento válido
        except ValueError as e:
            if "fora dos limites" in str(e):
                print("GAME OVER! Carro saiu dos limites do mapa!")
            elif "obstáculo" in str(e):
                print("GAME OVER! Carro bateu em um obstáculo!")
            else:
                print(f"GAME OVER! {e}")
            return False  # Encerra o jogo

def main():
    grid = Mapa()
    grid.mostrar()
    
    # Exemplo de como mover o carro
    print("\nComandos disponíveis: 'cima', 'baixo', 'esquerda', 'direita', 'sair'")
    print("Objetivo: Navegue sem bater nos obstáculos ou sair do mapa!")
    
    while True:
        comando = input("\nDigite um comando: ").strip().lower()
        
        if comando == 'sair':
            print("Saindo...")
            break
        elif comando in ['cima', 'baixo', 'esquerda', 'direita']:
            # Se mover_carro retorna False, o jogo deve encerrar
            if not grid.mover_carro(comando):
                grid.mostrar()  # Mostra o estado final
                print("\n🎮 Jogo encerrado!")
                break
            else:
                grid.mostrar()  # Mostra o novo estado apenas se movimento foi válido
        else:
            print("Comando inválido! Use: 'cima', 'baixo', 'esquerda', 'direita', 'sair'")

if __name__=="__main__":
    main()