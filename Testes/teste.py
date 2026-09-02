from ultralytics import YOLO

def testar_modelos_nas_fotos():
    # 1. Define os caminhos exatos dos arquivos best.pt que você treinou
    caminho_v8 = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/modelos_IA/YOLOv8n_Armadilha/weights/best.pt"
    caminho_v11 = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/modelos_IA/YOLOv11n_Armadilha/weights/best.pt"

    # 2. Define a pasta onde estão as fotos que você acabou de tirar
    pasta_fotos = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/tirar_foto/fotos_manuais"

    # 3. Carrega os modelos na memória
    print("\n[+] Carregando os pesos do YOLOv8n e YOLOv11n...")
    modelo_v8 = YOLO(caminho_v8)
    modelo_v11 = YOLO(caminho_v11)

    # 4. Executa a inferência na pasta inteira e salva os resultados
    print("\n" + "="*50)
    print(" INICIANDO TESTE COM O YOLOv8n")
    print("="*50)
    # O parâmetro save=True obriga o YOLO a desenhar a caixa na foto e salvar no disco
    modelo_v8.predict(
        source=pasta_fotos, 
        save=True, 
        project="/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/Comparacao_Visual", 
        name="Teste_v8n"
    )

    print("\n" + "="*50)
    print(" INICIANDO TESTE COM O YOLOv11n")
    print("="*50)
    modelo_v11.predict(
        source=pasta_fotos, 
        save=True, 
        project="/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/Comparacao_Visual", 
        name="Teste_v11n"
    )

    print("\n[+] Teste concluído com sucesso!")
    print(" -> Vá até a pasta /Code/Comparacao_Visual/ para ver as imagens com as caixas desenhadas e comparar as detecções.")

if __name__ == "__main__":
    testar_modelos_nas_fotos()
