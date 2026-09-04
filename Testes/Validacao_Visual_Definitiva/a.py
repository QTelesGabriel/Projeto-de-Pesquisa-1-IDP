from ultralytics import YOLO
import os

def comparar_modelos_visualmente():
    # 1. Caminhos absolutos dos pesos finais que você acabou de treinar
    caminho_v8 = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/modelos_IA/YOLOv8n_Armadilha/weights/best.pt"
    caminho_v11 = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/modelos_IA/YOLOv11n_Armadilha/weights/best.pt"

    # 2. Caminho exato da pasta onde você tirou as fotos no Gazebo usando a tecla 'C'
    pasta_fotos_manuais = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/ferramentas/tirar_foto/fotos_manuais"
    pasta_destino_resultados = "/home/gabriel/Projetos/Projeto-de-Pesquisa-1-IDP/Code/Validacao_Visual_Definitiva"

    print("\n[+] Carregando os modelos YOLOv8n e YOLOv11n treinados do zero...")
    
    try:
        modelo_v8 = YOLO(caminho_v8)
        modelo_v11 = YOLO(caminho_v11)
    except Exception as e:
        print(f"Erro ao carregar os modelos: {e}")
        return

    print("\n" + "="*50)
    print(" INICIANDO TESTE COM O YOLOv8n")
    print("="*50)
    # Roda a inferência do v8n e salva as imagens
    modelo_v8.predict(
        source=pasta_fotos_manuais, 
        save=True, 
        project=pasta_destino_resultados, 
        name="Resultados_v8n"
    )

    print("\n" + "="*50)
    print(" INICIANDO TESTE COM O YOLOv11n")
    print("="*50)
    # Roda a inferência do v11n e salva as imagens
    modelo_v11.predict(
        source=pasta_fotos_manuais, 
        save=True, 
        project=pasta_destino_resultados, 
        name="Resultados_v11n"
    )

    print("\n" + "="*50)
    print("[+] Teste e comparação concluídos com sucesso!")
    print(f" -> Verifique as imagens com as caixas na pasta: {pasta_destino_resultados}")

if __name__ == "__main__":
    comparar_modelos_visualmente()
