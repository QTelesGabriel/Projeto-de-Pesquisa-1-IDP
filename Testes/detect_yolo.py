from ultralytics import YOLO
import cv2

def main():
    # 1. Carrega o modelo YOLOv8n
    model = YOLO("yolov8n.pt")

    # 2. Inicializa a captura da Webcam
    # O argumento '0' geralmente é a câmera padrão do notebook/desktop
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Erro: Não foi possível acessar a webcam.")
        return

    print("Rodando YOLOv8n ao vivo. Pressione 'q' para sair.")

    while True:
        # Captura frame por frame
        success, frame = cap.read()

        if not success:
            break

        # 3. Executa a predição no frame atual
        # stream=True é mais eficiente para vídeos/webcam
        results = model.predict(source=frame, conf=0.5, stream=True)

        # 4. Processa e exibe o resultado
        for r in results:
            # O método .plot() desenha as caixas e nomes no frame
            annotated_frame = r.plot()
            
            # Exibe o frame anotado em uma janela
            cv2.imshow("YOLOv8 Webcam - Ao Vivo", annotated_frame)

        # 5. Condição de saída: pressionar a tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Limpeza após sair do loop
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()