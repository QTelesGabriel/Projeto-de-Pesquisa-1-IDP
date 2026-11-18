#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Menu Central de Execução da Suíte de Testes (TCC IDP)
Permite executar de forma simples e rápida cada um dos testes vinculados
aos 5 Objetivos Específicos da Monografia.
"""

import os
import sys
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def executar_script(nome_script, argumentos=None):
    caminho = os.path.join(SCRIPT_DIR, nome_script)
    if not os.path.exists(caminho):
        print(f"[!] Erro: Script {nome_script} não encontrado em {SCRIPT_DIR}")
        return
    cmd = [sys.executable, caminho]
    if argumentos:
        cmd.extend(argumentos)
    print(f"\n[+] Executando: {' '.join(cmd)}\n")
    subprocess.run(cmd)


def menu():
    while True:
        print("\n" + "="*75)
        print("🚀 SUÍTE DE TESTES E RESULTADOS CIENTÍFICOS DO TCC - IDP")
        print("="*75)
        print("  [1] TESTE 1: Benchmark YOLO (v8n, v10n, v11n, v26n em 2 Fases)")
        print("      -> Calcula P, R, mAP@50, mAP@50-95, FPS e gera Tabelas LaTeX")
        print("  [2] TESTE 1 (Modo Rápido): Extração instantânea das métricas dos treinos")
        print("  [3] TESTE 2: Validação Estocástica do Filtro de Kalman EKF")
        print("      -> Atenuação de Jitter e Predição Cega por 4.0s (Gera Gráfico)")
        print("  [4] TESTES 3, 4 e 5: Consolidador de Monte Carlo e Análise de Voo")
        print("      -> Retenção FOV, Alinhamento de Yaw, RMSE e Taxa de Acoplamento")
        print("  [5] EXECUTAR TODOS OS TESTES CIENTÍFICOS (Gera todos os resultados)")
        print("  [0] Sair")
        print("="*75)
        
        try:
            opcao = input(">> Escolha uma opção (0 a 5): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nEncerrando...")
            break
            
        if opcao == '1':
            executar_script('teste_1_benchmark_yolo.py')
        elif opcao == '2':
            executar_script('teste_1_benchmark_yolo.py', ['--rapido'])
        elif opcao == '3':
            executar_script('teste_2_validacao_ekf.py')
        elif opcao == '4':
            executar_script('teste_3_4_5_analise_monte_carlo.py')
        elif opcao == '5':
            print("\n>>> INICIANDO BATERIA COMPLETA DE TESTES CIENTÍFICOS <<<\n")
            executar_script('teste_1_benchmark_yolo.py', ['--rapido'])
            executar_script('teste_2_validacao_ekf.py')
            executar_script('teste_3_4_5_analise_monte_carlo.py')
            print("\n>>> TODOS OS RESULTADOS, TABELAS LATEX E GRÁFICOS FORAM GERADOS! <<<")
            print(f"    Consulte as pastas:")
            print(f"      * Tabelas e CSVs : {os.path.join(SCRIPT_DIR, 'resultados')}")
            print(f"      * Gráficos em Alta: {os.path.join(SCRIPT_DIR, 'graficos')}\n")
        elif opcao == '0':
            print("\nAté logo!")
            break
        else:
            print("\n[!] Opção inválida. Digite um número de 0 a 5.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == '--all':
        executar_script('teste_1_benchmark_yolo.py', ['--rapido'])
        executar_script('teste_2_validacao_ekf.py')
        executar_script('teste_3_4_5_analise_monte_carlo.py')
    else:
        menu()
