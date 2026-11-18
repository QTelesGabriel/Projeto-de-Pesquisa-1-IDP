# Suíte de Testes e Extração de Resultados Científicos (TCC IDP)

Este diretório contém a suíte padronizada e automatizada de testes do projeto de pesquisa **"Autonomous Object Capture System By UAVs Using Visual Servoing"**, rigorosamente estruturada para validar os **5 Objetivos Específicos** descritos na introdução da monografia.

---

## 🎯 Mapeamento dos Testes vs. Objetivos da Monografia

| Teste | Objetivo Específico | Escopo de Validação | Script Responsável | Arquivos de Saída |
|---|---|---|---|---|
| **Teste 1** | **Objetivo 1:** Comparar 4 gerações YOLO (v8n, v10n, v11n e v26n) em 2 fases ($mAP@50 > 90\%$, $\text{FPS} \ge 20$) | Acurácia ($P, R, mAP$), Latência ($\text{ms}$) e $\text{FPS}$ nos datasets das Fases 1 e 2 | `teste_1_benchmark_yolo.py` | `resultados/benchmark_yolo_resultados.csv`<br>`resultados/tabelas_yolo_artigo.tex` |
| **Teste 2** | **Objetivo 2:** Filtro de Kalman EKF com atenuação de *jitter* e predição cega por até $4,0\text{ s}$ | Redução da variância de ruído da visão e sustentação preditiva durante oclusão | `teste_2_validacao_ekf.py` | `resultados/validacao_ekf_metricas.csv`<br>`graficos/validacao_ekf_4segundos.png` |
| **Teste 3** | **Objetivo 3:** Conversão Pinhole 3D e Histerese ($40\%$ segura, corte $> 60\%$, $100\%$ retido no FOV) | Contenção contínua da armadilha no enquadramento óptico durante descida em funil | `Code/missao/main.py`<br>`teste_3_4_5_analise_monte_carlo.py` | `Code/missao/telemetria/` |
| **Teste 4** | **Objetivo 4:** Controle em cascata (400 Hz / 20 Hz) e rotação autônoma com $|e_\psi| \le 1,0^\circ$ | Estabilidade de atitude e alinhamento terminal de guinada a $1,50\text{ m}$ | `Code/missao/main.py`<br>`teste_3_4_5_analise_monte_carlo.py` | `graficos/perfil_voo_missao_completa.png` |
| **Teste 5** | **Objetivo 5:** Missão completa Monte Carlo, ancoragem milimétrica ($\le 8\text{ mm}$ por 1,5s), *Blind Drop* e sucesso $> 60\%$ | Repetibilidade estatística sob dispersão estocástica de GPS civil ($1,0$ a $4,5\text{ m}$) | `Code/missao/main.py`<br>`teste_3_4_5_analise_monte_carlo.py` | `Code/missao/relatorio_testes_monte_carlo.csv`<br>`resultados/tabela_monte_carlo_artigo.tex` |

---

## 🚀 Como Executar os Testes

### Opção 1: Menu Interativo Central (Recomendado)
Para acessar o menu interativo e disparar qualquer teste individualmente ou em lote:
```bash
python3 Testes/teste.py
```

### Opção 2: Linha de Comando Direta

#### 1. Teste 1 - Benchmark YOLO (Modo Instantâneo dos Treinamentos)
```bash
python3 Testes/teste_1_benchmark_yolo.py --rapido
```
*Gera instantaneamente a tabela comparativa das 4 redes e o código LaTeX para o artigo.*

#### 2. Teste 1 - Benchmark YOLO ao Vivo (com medição de FPS na GPU/CPU)
```bash
python3 Testes/teste_1_benchmark_yolo.py
```

#### 3. Teste 2 - Validação Estocástica do Filtro de Kalman (EKF)
```bash
python3 Testes/teste_2_validacao_ekf.py
```
*Gera a figura científica de atenuação de tremor e predição cega por 4 segundos.*

#### 4. Testes 3, 4 e 5 - Consolidador de Voos de Monte Carlo
```bash
python3 Testes/teste_3_4_5_analise_monte_carlo.py
```
*Processa o histórico de voos simulados, calcula a taxa de acoplamento real e plota a curva temporal de altitude e erros.*

---

## 🚁 Como Rodar as Simulações em Modo Semiautomático

Ao rodar missões de simulação no Gazebo:

1. **Inicie o Ambiente (Terminais 1 a 3 conforme `Textos/como_ligar.txt`):**
   - Terminal 1: Gazebo Harmonic (`worlds/tcc.sdf`)
   - Terminal 2: ArduPilot SITL (`sim_vehicle.py -v ArduCopter -f octa-quad ...`)
   - Terminal 3: Ponte ROS 2 (`ros_gz_bridge`)

2. **Terminal 4 - Dispare a Missão:**
   ```bash
   cd ~/Projetos/Projeto-de-Pesquisa-1-IDP/Code/missao
   python3 main.py
   ```

3. **Automação no Encerramento do Voo:**
   - O sistema grava automaticamente a série temporal do voo em `Code/missao/telemetria/telemetria_voo_<timestamp>.csv`.
   - Ao pousar, o algoritmo avalia os critérios analíticos do TCC ($\text{RMSE} \le 8,0\text{ mm}$, $|e_\psi| \le 1,0^\circ$ e FOV retido).
   - O voo é **salvo automaticamente no CSV em 5 segundos**, a menos que você digite `N` (forçar falha) ou `D` (descartar).
   - Para rodar várias missões de Monte Carlo, basta reiniciar o Terminal 4 para cada novo voo sorteado estocasticamente.

---

## 📁 Estrutura de Arquivos Gerados

- `Testes/resultados/`
  - `benchmark_yolo_resultados.csv`: métricas de acurácia, latência e FPS das 4 redes em ambas as fases.
  - `tabelas_yolo_artigo.tex`: código LaTeX pronto para inserção no artigo (Tabelas da Fase 1 e Fase 2).
  - `validacao_ekf_metricas.csv`: métricas de redução de ruído e deriva de predição do EKF.
  - `tabela_monte_carlo_artigo.tex`: tabela consolidada de Monte Carlo em LaTeX.
- `Testes/graficos/`
  - `validacao_ekf_4segundos.png`: gráfico em alta resolução mostrando atenuação de jitter e a janela de 4s.
  - `perfil_voo_missao_completa.png`: gráfico da evolução vertical da aeronave e convergência dos erros.
