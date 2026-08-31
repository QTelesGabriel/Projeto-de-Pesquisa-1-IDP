import os
import shutil

def organizar_dataset_yolo():
    # Caminho base do dataset
    base_origem = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/yolo_label/dataset"
    
    # Caminho onde as imagens realmente estão
    pasta_imagens = os.path.join(base_origem, "imagens")
    
    # Caminho principal onde o dataset oficial do YOLO será criado
    destino_base = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/yolo_dataset"
    
    # Cria a estrutura de pastas exigida pelo YOLO
    pastas = [
        "images/train", "images/val",
        "labels/train", "labels/val"
    ]
    
    for pasta in pastas:
        caminho_completo = os.path.join(destino_base, pasta)
        os.makedirs(caminho_completo, exist_ok=True)
        
    # Pega todos os arquivos .jpg diretamente de dentro da pasta 'imagens'
    try:
        arquivos = sorted(os.listdir(pasta_imagens))
    except FileNotFoundError:
        print(f"Erro: A pasta {pasta_imagens} não foi encontrada. Verifique o caminho.")
        return
        
    imagens = [f for f in arquivos if f.endswith('.jpg')]
    
    if not imagens:
        print(f"Nenhuma imagem .jpg foi encontrada em {pasta_imagens}.")
        return

    contador_train = 0
    contador_val = 0
    
    print(f"Encontradas {len(imagens)} imagens. Copiando e organizando...")
    
    for i, imagem in enumerate(imagens):
        nome_base = imagem.replace('.jpg', '')
        arquivo_txt = f"{nome_base}.txt"
        
        caminho_img_origem = os.path.join(base_origem, "imagens", imagem)
        caminho_txt_origem = os.path.join(base_origem, "labels", arquivo_txt)
        
        # Ignora se a imagem não tiver o TXT correspondente
        if not os.path.exists(caminho_txt_origem):
            print(f"[Aviso] {imagem} não tem arquivo .txt correspondente. Ignorada.")
            continue
            
        # Lógica de separação: A cada 4 imagens, o índice (0, 1, 2, 3) % 4 == 3 vai para validação
        if i % 4 == 3:
            subpasta = "val"
            contador_val += 1
        else:
            subpasta = "train"
            contador_train += 1
            
        # Define os caminhos de destino
        caminho_img_destino = os.path.join(destino_base, "images", subpasta, imagem)
        caminho_txt_destino = os.path.join(destino_base, "labels", subpasta, arquivo_txt)
        
        # Copia os arquivos
        shutil.copy(caminho_img_origem, caminho_img_destino)
        shutil.copy(caminho_txt_origem, caminho_txt_destino)
        
    print(f"\n[+] Sucesso! Dataset YOLO criado em: {destino_base}")
    print(f" -> Imagens de Treino: {contador_train}")
    print(f" -> Imagens de Validação (Teste): {contador_val}")

if __name__ == "__main__":
    organizar_dataset_yolo()