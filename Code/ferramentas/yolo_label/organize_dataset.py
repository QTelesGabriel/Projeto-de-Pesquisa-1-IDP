import os
import shutil

def atualizar_dataset_yolo():
    # Caminho base do dataset (onde estão TODAS as fotos e txts misturados)
    base_origem = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/yolo_label/dataset"
    pasta_imagens = os.path.join(base_origem, "imagens")
    
    # Caminho principal onde o dataset oficial do YOLO já existe
    destino_base = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/yolo_dataset"
    
    # Garante que as pastas de destino existem (caso alguma tenha sido apagada)
    pastas = [
        "images/train", "images/val",
        "labels/train", "labels/val"
    ]
    for pasta in pastas:
        os.makedirs(os.path.join(destino_base, pasta), exist_ok=True)
        
    try:
        arquivos_origem = sorted(os.listdir(pasta_imagens))
    except FileNotFoundError:
        print(f"Erro: A pasta {pasta_imagens} não foi encontrada.")
        return
        
    imagens_origem = [f for f in arquivos_origem if f.endswith('.jpg')]
    
    # ---------------------------------------------------------
    # NOVIDADE: Verifica o que já existe no dataset
    # ---------------------------------------------------------
    imagens_treino_existentes = set(os.listdir(os.path.join(destino_base, "images", "train")))
    imagens_val_existentes = set(os.listdir(os.path.join(destino_base, "images", "val")))
    todas_existentes = imagens_treino_existentes.union(imagens_val_existentes)
    
    # Filtra apenas as imagens que AINDA NÃO estão no destino
    novas_imagens = [img for img in imagens_origem if img not in todas_existentes]
    
    if not novas_imagens:
        print("Nenhuma imagem nova encontrada para copiar. O dataset já está atualizado.")
        return

    print(f"Encontradas {len(novas_imagens)} NOVAS imagens. Iniciando a cópia organizada...")
    
    contador_train = 0
    contador_val = 0
    
    # O loop agora roda APENAS nas imagens novas
    for i, imagem in enumerate(novas_imagens):
        nome_base = imagem.replace('.jpg', '')
        arquivo_txt = f"{nome_base}.txt"
        
        caminho_img_origem = os.path.join(base_origem, "imagens", imagem)
        caminho_txt_origem = os.path.join(base_origem, "labels", arquivo_txt)
        
        if not os.path.exists(caminho_txt_origem):
            print(f"[Aviso] {imagem} não tem arquivo .txt correspondente. Ignorada.")
            continue
            
        # Mantém a proporção 75/25 para o lote novo
        if i % 4 == 3:
            subpasta = "val"
            contador_val += 1
        else:
            subpasta = "train"
            contador_train += 1
            
        caminho_img_destino = os.path.join(destino_base, "images", subpasta, imagem)
        caminho_txt_destino = os.path.join(destino_base, "labels", subpasta, arquivo_txt)
        
        # Copia os arquivos novos
        shutil.copy(caminho_img_origem, caminho_img_destino)
        shutil.copy(caminho_txt_origem, caminho_txt_destino)
        
    print(f"\n[+] Atualização do Dataset concluída com sucesso!")
    print(f" -> Novas Imagens em Treino: {contador_train}")
    print(f" -> Novas Imagens em Validação (Teste): {contador_val}")
    print(f" -> Total no Dataset agora: {len(todas_existentes) + len(novas_imagens)} imagens")

if __name__ == "__main__":
    atualizar_dataset_yolo()