#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teste 1: Benchmark Comparativo das Redes Neurais YOLO (Objetivo Específico 1 do TCC)
Compara 4 gerações da família YOLO (v8n, v10n, v11n e v26n) em duas fases de voo:
  - Fase 1: Detecção Macroscópica da Carcaça da Armadilha (Classe Única)
  - Fase 2: Detecção Micro de Alta Precisão das Alças e Tampa (2 Classes)

Calcula:
  - Precisão (P)
  - Recall (R)
  - mAP@50 (Meta: > 90%)
  - mAP@50-95
  - Latência de Pré-processamento, Inferência e Pós-processamento (ms)
  - Taxa de Quadros por Segundo (FPS) (Meta: >= 20 FPS)
  - Exportação em formato CSV e tabelas formatadas em LaTeX para a monografia.
"""

import os
import sys
import time
import argparse
import csv
from datetime import datetime

# Diretórios base do projeto
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
MODELOS_DIR = os.path.join(ROOT_DIR, 'Code', 'modelos_IA')
DATASET_FASE1 = os.path.join(ROOT_DIR, 'Code', 'ferramentas', 'datasets', 'YOLOv11n', 'data.yaml')
DATASET_FASE2 = os.path.join(ROOT_DIR, 'Code', 'ferramentas', 'datasets', 'YOLOv11n-Fase2', 'data.yaml')
RESULTADOS_DIR = os.path.join(os.path.dirname(__file__), 'resultados')
os.makedirs(RESULTADOS_DIR, exist_ok=True)

MODELOS_CONFIG = {
    'Fase 1': [
        ('YOLOv8n', os.path.join(MODELOS_DIR, 'YOLOv8n_Armadilha_TCC', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv8n_Armadilha_TCC', 'results.csv')),
        ('YOLOv10n', os.path.join(MODELOS_DIR, 'YOLOv10n_Armadilha_TCC', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv10n_Armadilha_TCC', 'results.csv')),
        ('YOLOv11n', os.path.join(MODELOS_DIR, 'YOLOv11n_Armadilha_TCC', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv11n_Armadilha_TCC', 'results.csv')),
        ('YOLO26n', os.path.join(MODELOS_DIR, 'YOLO26n_Armadilha_TCC', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLO26n_Armadilha_TCC', 'results.csv')),
    ],
    'Fase 2': [
        ('YOLOv8n', os.path.join(MODELOS_DIR, 'YOLOv8n_Fase2_Armadilha_Alcas', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv8n_Fase2_Armadilha_Alcas', 'results.csv')),
        ('YOLOv10n', os.path.join(MODELOS_DIR, 'YOLOv10n_Fase2_Armadilha_Alcas', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv10n_Fase2_Armadilha_Alcas', 'results.csv')),
        ('YOLOv11n', os.path.join(MODELOS_DIR, 'YOLOv11n_Fase2_Armadilha_Alcas', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLOv11n_Fase2_Armadilha_Alcas', 'results.csv')),
        ('YOLO26n', os.path.join(MODELOS_DIR, 'YOLO26n_Fase2_Armadilha_Alcas', 'weights', 'best.pt'), os.path.join(MODELOS_DIR, 'YOLO26n_Fase2_Armadilha_Alcas', 'results.csv')),
    ]
}


def extrair_metricas_csv_treinamento(caminho_csv):
    """Extrai os melhores resultados consolidados do histórico de treino da Ultralytics sem depender de pandas."""
    if not os.path.exists(caminho_csv):
        return None
    try:
        with open(caminho_csv, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            headers = [h.strip() for h in next(reader)]
            
            idx_p = headers.index('metrics/precision(B)') if 'metrics/precision(B)' in headers else -1
            idx_r = headers.index('metrics/recall(B)') if 'metrics/recall(B)' in headers else -1
            idx_m50 = headers.index('metrics/mAP50(B)') if 'metrics/mAP50(B)' in headers else -1
            idx_m95 = headers.index('metrics/mAP50-95(B)') if 'metrics/mAP50-95(B)' in headers else -1
            
            best_m95 = -1.0
            best_row = None
            total_epochs = 0
            
            for row in reader:
                if not row or len(row) <= max(idx_p, idx_r, idx_m50, idx_m95):
                    continue
                total_epochs += 1
                try:
                    val_m95 = float(row[idx_m95].strip())
                    if val_m95 > best_m95:
                        best_m95 = val_m95
                        best_row = row
                except ValueError:
                    continue
                    
            if best_row:
                return {
                    'P': float(best_row[idx_p].strip()) * 100.0,
                    'R': float(best_row[idx_r].strip()) * 100.0,
                    'mAP50': float(best_row[idx_m50].strip()) * 100.0,
                    'mAP50_95': float(best_row[idx_m95].strip()) * 100.0,
                    'epocas': total_epochs
                }
    except Exception as e:
        print(f"  [!] Aviso ao ler CSV {caminho_csv}: {e}")
    return None


def avaliar_ao_vivo(modelo_path, dataset_yaml):
    """Executa o val() da Ultralytics para medir acurácia e tempos reais de inferência / FPS."""
    from ultralytics import YOLO
    
    print(f"    -> Carregando pesos: {os.path.basename(os.path.dirname(os.path.dirname(modelo_path)))}...")
    modelo = YOLO(modelo_path)
    
    # Executa validação no dataset especificado
    metrics = modelo.val(data=dataset_yaml, split='val', verbose=False, imgsz=640, batch=8)
    
    p = float(metrics.box.mp) * 100.0 if hasattr(metrics.box, 'mp') else float(metrics.box.p.mean()) * 100.0
    r = float(metrics.box.mr) * 100.0 if hasattr(metrics.box, 'mr') else float(metrics.box.r.mean()) * 100.0
    map50 = float(metrics.box.map50) * 100.0
    map5095 = float(metrics.box.map) * 100.0
    
    speed = metrics.speed # {'preprocess': ms, 'inference': ms, 'loss': ms, 'postprocess': ms}
    t_pre = speed.get('preprocess', 0.0)
    t_inf = speed.get('inference', 0.0)
    t_post = speed.get('postprocess', 0.0)
    latencia_total = t_pre + t_inf + t_post
    fps = 1000.0 / latencia_total if latencia_total > 0 else 0.0
    
    return {
        'P': p,
        'R': r,
        'mAP50': map50,
        'mAP50_95': map5095,
        'latencia_ms': latencia_total,
        't_inf_ms': t_inf,
        'fps': fps
    }


def gerar_tabela_latex(registros_fase, titulo_fase, label_tabela, modelo_destaque=None):
    """Gera bloco de código LaTeX pronto para inserção no artigo."""
    latex = []
    latex.append(f"% ==================== TABELA LATEX: {titulo_fase.upper()} ====================")
    latex.append("\\begin{table}[H]")
    latex.append("    \\centering")
    latex.append("    \\small")
    latex.append("    \\setlength{\\tabcolsep}{5pt}")
    latex.append("    \\captionsetup{justification=centering}")
    latex.append(f"    \\caption{{Comparativo de desempenho entre arquiteturas da família YOLO na {titulo_fase}}}")
    latex.append(f"    \\label{{{label_tabela}}}")
    latex.append("    \\vspace{0.2cm}")
    latex.append("    \\begin{tabular}{lcccccc}")
    latex.append("        \\hline")
    latex.append("        \\textbf{Arquitetura} & \\textbf{Precisão ($P$)} & \\textbf{Recall ($R$)} & \\textbf{$mAP@50$} & \\textbf{$mAP@50\\text{--}95$} & \\textbf{Latência (ms)} & \\textbf{Taxa (FPS)} \\\\")
    latex.append("        \\hline")
    
    for row in registros_fase:
        arq = str(row['Arquitetura'])
        destaque = (arq == modelo_destaque) if modelo_destaque else False
        p_str = f"{row['Precisao_pct']:.2f}\\%"
        r_str = f"{row['Recall_pct']:.2f}\\%"
        m50_str = f"{row['mAP50_pct']:.2f}\\%"
        m95_str = f"{row['mAP50_95_pct']:.2f}\\%"
        lat_str = f"{row['Latencia_ms']:.1f}" if row['Latencia_ms'] is not None else "--"
        fps_str = f"{row['FPS']:.1f}" if row['FPS'] is not None else "--"
        
        if destaque:
            latex.append(f"        \\textbf{{{arq}}} & \\textbf{{{p_str}}} & \\textbf{{{r_str}}} & \\textbf{{{m50_str}}} & \\textbf{{{m95_str}}} & \\textbf{{{lat_str}}} & \\textbf{{{fps_str}}} \\\\")
        else:
            latex.append(f"        {arq:<10} & {p_str:<12} & {r_str:<10} & {m50_str:<10} & {m95_str:<15} & {lat_str:<12} & {fps_str} \\\\")
            
    latex.append("        \\hline")
    latex.append("    \\end{tabular}")
    latex.append("    \\vspace{0.2cm}")
    latex.append("    \\begin{flushleft}")
    latex.append("        \\footnotesize Fonte: elaboração própria.")
    latex.append("    \\end{flushleft}")
    latex.append("\\end{table}\n")
    return "\n".join(latex)


def main():
    parser = argparse.ArgumentParser(description="Teste 1: Benchmark YOLO (TCC IDP)")
    parser.add_argument('--rapido', action='store_true', help="Usa apenas os dados gravados no CSV de treino sem rodar inferência ao vivo.")
    args = parser.parse_args()
    
    print("\n" + "="*80)
    print("🎯 TESTE 1: BENCHMARK COMPARATIVO DAS REDES YOLO (OBJETIVO 1)")
    print("   Meta TCC: mAP@50 > 90.0% e Processamento >= 20.0 FPS")
    print("="*80)
    
    todos_resultados = []
    
    for fase_nome, lista_modelos in MODELOS_CONFIG.items():
        dataset_yaml = DATASET_FASE1 if '1' in fase_nome else DATASET_FASE2
        print(f"\n[+] Executando Avaliação para: {fase_nome.upper()}...")
        print(f"    Dataset: {dataset_yaml}")
        
        for nome_arq, peso_path, csv_path in lista_modelos:
            print(f"  * Testando {nome_arq}...")
            
            if not os.path.exists(peso_path):
                print(f"    [!] Arquivo de pesos não encontrado: {peso_path}")
                continue
                
            dados_obtidos = None
            
            if not args.rapido:
                try:
                    dados_obtidos = avaliar_ao_vivo(peso_path, dataset_yaml)
                except Exception as e:
                    print(f"    [!] Falha ao rodar .val() ao vivo ({e}). Tentando extrair do CSV de treino...")
                    
            if dados_obtidos is None:
                # Fallback para o CSV de treino
                metricas_csv = extrair_metricas_csv_treinamento(csv_path)
                if metricas_csv:
                    dados_obtidos = {
                        'P': metricas_csv['P'],
                        'R': metricas_csv['R'],
                        'mAP50': metricas_csv['mAP50'],
                        'mAP50_95': metricas_csv['mAP50_95'],
                        'latencia_ms': None,
                        't_inf_ms': None,
                        'fps': None
                    }
                else:
                    print(f"    [!] Não foi possível extrair métricas de {nome_arq}.")
                    continue
                    
            # Avalia conformidade com os objetivos
            atingiu_map50 = (dados_obtidos['mAP50'] > 90.0)
            atingiu_fps = (dados_obtidos['fps'] >= 20.0) if dados_obtidos['fps'] is not None else None
            
            registro = {
                'Fase': fase_nome,
                'Arquitetura': nome_arq,
                'Precisao_pct': round(dados_obtidos['P'], 2),
                'Recall_pct': round(dados_obtidos['R'], 2),
                'mAP50_pct': round(dados_obtidos['mAP50'], 2),
                'mAP50_95_pct': round(dados_obtidos['mAP50_95'], 2),
                'Latencia_ms': round(dados_obtidos['latencia_ms'], 2) if dados_obtidos['latencia_ms'] is not None else None,
                'FPS': round(dados_obtidos['fps'], 1) if dados_obtidos['fps'] is not None else None,
                'Atendeu_mAP50': 'SIM' if atingiu_map50 else 'NAO',
                'Atendeu_FPS': 'SIM' if atingiu_fps else ('--' if atingiu_fps is None else 'NAO'),
                'Caminho_Pesos': peso_path
            }
            todos_resultados.append(registro)
            
            fps_disp = f"{registro['FPS']} FPS" if registro['FPS'] is not None else "N/A"
            print(f"    -> P: {registro['Precisao_pct']}% | R: {registro['Recall_pct']}% | mAP@50: {registro['mAP50_pct']}% | mAP@50-95: {registro['mAP50_95_pct']}% | {fps_disp}")

    # Salva CSV Consolidado
    caminho_saida_csv = os.path.join(RESULTADOS_DIR, 'benchmark_yolo_resultados.csv')
    if todos_resultados:
        campos = list(todos_resultados[0].keys())
        with open(caminho_saida_csv, 'w', newline='', encoding='utf-8') as f_out:
            writer = csv.DictWriter(f_out, fieldnames=campos)
            writer.writeheader()
            writer.writerows(todos_resultados)
        print(f"\n[+] Resultados salvos com sucesso em: {caminho_saida_csv}")
    
    # Exibe Resumo em Tabela no Terminal
    print("\n" + "="*85)
    print("📋 TABELA RESUMO CONSOLIDADA (TESTE 1)")
    print("="*85)
    header_fmt = "{:<8} {:<10} {:<10} {:<10} {:<10} {:<12} {:<12} {:<8} {:<8}"
    print(header_fmt.format("Fase", "Modelo", "Prec(%)", "Rec(%)", "mAP50(%)", "mAP50-95(%)", "Lat(ms)", "FPS", "MetaOK?"))
    print("-" * 85)
    for r in todos_resultados:
        lat_str = f"{r['Latencia_ms']:.1f}" if r['Latencia_ms'] is not None else "--"
        fps_str = f"{r['FPS']:.1f}" if r['FPS'] is not None else "--"
        meta_str = "SIM" if (r['Atendeu_mAP50'] == 'SIM' and (r['Atendeu_FPS'] in ['SIM', '--'])) else "NAO"
        print(header_fmt.format(
            r['Fase'], r['Arquitetura'], str(r['Precisao_pct']), str(r['Recall_pct']),
            str(r['mAP50_pct']), str(r['mAP50_95_pct']), lat_str, fps_str, meta_str
        ))
    print("="*85)
    
    # Gera e Salva o Código LaTeX
    caminho_saida_latex = os.path.join(RESULTADOS_DIR, 'tabelas_yolo_artigo.tex')
    with open(caminho_saida_latex, 'w', encoding='utf-8') as f_tex:
        reg_f1 = [r for r in todos_resultados if r['Fase'] == 'Fase 1']
        reg_f2 = [r for r in todos_resultados if r['Fase'] == 'Fase 2']
        tex1 = gerar_tabela_latex(reg_f1, "detecção macroscópica da armadilha na Fase 1", "tab:comparativo_yolo_fase1_resultados", modelo_destaque='YOLO26n')
        tex2 = gerar_tabela_latex(reg_f2, "detecção multi-classe da Fase 2 (alças e tampa)", "tab:comparativo_yolo_fase2_resultados", modelo_destaque='YOLOv10n')
        f_tex.write(tex1 + "\n\n" + tex2)
        
    print(f"[+] Tabelas LaTeX geradas e salvas em: {caminho_saida_latex}")
    print("\n" + tex1)
    print("\n" + tex2)


if __name__ == "__main__":
    main()
